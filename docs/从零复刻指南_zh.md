# 智感环境助手 · 从零复刻指南

> 目标:基于开源小智语音助手(xiaozhi-esp32),从零构建一个"语音控制 + 温湿度检测 + 风扇联动"的智能环境感知终端。
> 主控:ESP32-P4(ESP-P4-Function-EV-Board) | 工程目标:esp32p4
> 预计总工时:按功能分步,每步 1–4 小时,全程约 20–30 小时。

---

## 前置准备:硬件清单

| 硬件 | 数量 | 用途 |
| --- | --- | --- |
| ESP-P4-Function-EV-Board | 1 | 核心主控(含 LCD/触摸/音频/摄像头/SD 卡) |
| DHT11 温湿度传感器 | 1 | 环境感知 |
| L9110S 电机驱动模块 | 1 | 风扇驱动 |
| 小型直流风扇 | 1 | 执行器 |
| 杜邦线(公母) | 若干 | 接线 |
| USB-C 数据线 | 1 | 烧录与供电 |

> 如果使用其他 ESP32 板卡(如 ESP32-S3),需自行适配板级文件,本文以 ESP-P4-Function-EV-Board 为准。

---

## 第 0 步:ESP-IDF 开发环境搭建(约 2 小时)

### 0.1 安装 ESP-IDF

```powershell
# 方式一:VS Code + ESP-IDF 插件(推荐新手)
# 在 VS Code 扩展市场搜索 "ESP-IDF",安装后按向导配置。

# 方式二:命令行安装(推荐)
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2
.\install.ps1 esp32p4
```

### 0.2 验证环境

```powershell
.\export.ps1
idf.py --version
# 应输出 ESP-IDF v5.5.2
```

### 0.3 克隆基础工程

```powershell
git clone https://github.com/78/xiaozhi-esp32.git
cd xiaozhi-esp32
```

### 0.4 首次编译验证

```powershell
idf.py set-target esp32p4
# 在 menuconfig 中选择 Board Type → ESP-P4-Function-EV-Board
idf.py menuconfig
idf.py build
```

> 里程碑:固件编译通过,产生 build/xiaozhi.bin。

---

## 第 1 步:让基础框架跑起来(约 3 小时)

### 1.1 理解工程启动流程

main/main.cc → app_main() → Application::Initialize() → Application::Run()

Application::Initialize() 负责:
- 初始化 LVGL 显示 UI
- 初始化 AudioService(音频采集/播放)
- 注册唤醒词、VAD 回调
- 注册状态机回调
- 注册 MCP 通用工具
- 设置网络事件回调
- 启动网络连接

### 1.2 烧录并验证基础功能

```powershell
idf.py -p COM3 flash monitor
```

验证项:
- [ ] 屏幕正常显示(LVGL 界面)
- [ ] Wi-Fi 配网成功(热点配网/声波配网)
- [ ] 设备激活成功,进入空闲状态
- [ ] 按 BOOT 键能触发语音对话(需配置云端服务地址)

### 1.3 理解板级代码结构

```
main/boards/esp-p4-function-ev-board/
├── config.h                        # 引脚定义、音频参数
├── esp-p4-function-ev-board.cc     # 板级初始化 + MCP 工具注册
```

板级构造函数执行顺序:
1. I2C 总线初始化
2. MIPI LCD 初始化(1024×600)
3. BOOT 按键初始化
4. 触摸屏初始化(GT911)
5. SD 卡挂载
6. 摄像头初始化
7. 字体初始化
8. 风扇 GPIO 初始化——这是我们要添加的
9. MCP 工具注册——这是我们要扩展的

> 里程碑:基础框架正常运行,能语音对话。

---

## 第 2 步:加入 DHT11 温湿度传感器(约 4 小时)

### 2.1 硬件接线

```
DHT11        ESP32-P4
VCC    →     3.3V
GND    →     GND
DATA   →     GPIO5(建议加 4.7kΩ 上拉电阻)
```

### 2.2 创建自研 DHT11 驱动

在 components/sw_dht11/ 下创建驱动:

**文件结构:**
```
components/sw_dht11/
├── CMakeLists.txt
├── include/
│   └── sw_dht11.h
└── src/
    └── sw_dht11.c
```

**CMakeLists.txt:**
```cmake
idf_component_register(
    SRCS "src/sw_dht11.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_rom freertos
)
```

**sw_dht11.h 核心接口:**
```c
#pragma once
#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>

void dht11_set_pin(gpio_num_t pin);
gpio_num_t dht11_get_pin(void);
bool dht11_read(uint8_t *humidity, uint8_t *temperature);
const char *dht11_get_last_error(void);
```

**sw_dht11.c 实现要点:**

- 主机拉低 20ms 起始信号,然后释放总线,切换为输入模式
- 检测 DHT11 应答:先拉低约 80μs,再拉高约 80μs
- 逐位采样 40bit:湿度整数(8bit)+湿度小数(0)+温度整数(8bit)+温度小数(0)+校验和(8bit)
- 每 bit 判断:先等信号变高,延时 40μs 后读电平,高电平=1,低电平=0
- 读取完成后校验:前 4 字节和 = 第 5 字节,不匹配则返回错误
- **关键**:读取期间用 taskENTER_CRITICAL 关中断,防止时序被破坏

### 2.3 创建 DHT11 采集服务

在 main/ 下创建 dht11_telemetry_service.cc 和 .h:

**核心逻辑:**
```cpp
class Dht11TelemetryService {
public:
    void Start();   // 创建 FreeRTOS 任务
    void Stop();    // 安全停止任务
    bool GetLatestReading(int& temperature, int& humidity, int64_t& uptime_ms) const;

private:
    void TaskLoop() {
        while (started_.load()) {
            // 1. 初始化传感器(首次)
            if (!sensor_initialized_ && !InitializeSensor()) {
                WaitForStopOrDelay(5000ms);  // 失败等 5 秒重试
                continue;
            }
            // 2. 读取
            uint8_t temp, hum;
            if (!dht11_read(&hum, &temp)) {
                WaitForStopOrDelay(5000ms);
                continue;
            }
            // 3. 缓存到原子变量
            latest_temperature_.store(temp);
            latest_humidity_.store(hum);
            has_latest_reading_.store(true);
            // 4. 等待下一个周期
            WaitForStopOrDelay(60000ms);  // 默认 60 秒
        }
    }

    std::atomic<int> latest_temperature_{0};
    std::atomic<int> latest_humidity_{0};
    std::atomic<bool> has_latest_reading_{false};
};
```

### 2.4 在 Application 中集成

main/application.cc 中 Initialize() 末尾添加:
```cpp
#if CONFIG_ENABLE_DHT11_TELEMETRY
    dht11_telemetry_service_ = std::make_unique<Dht11TelemetryService>();
    dht11_telemetry_service_->Start();
#endif
```

### 2.5 添加 Kconfig 配置项

main/Kconfig.projbuild 末尾添加:
```kconfig
menu "DHT11 Telemetry"
    config ENABLE_DHT11_TELEMETRY
        bool "Enable DHT11 temperature/humidity readings"
        default n
    config DHT11_GPIO_NUM
        int "DHT11 data GPIO"
        default 5
    config DHT11_REPORT_INTERVAL_MS
        int "Telemetry interval (ms)"
        default 60000
        range 2000 3600000
endmenu
```

### 2.6 修改 CMakeLists.txt

main/CMakeLists.txt 的 SOURCES 列表中添加:
```cmake
"dht11_telemetry_service.cc"
```

### 2.7 验证

```powershell
idf.py menuconfig
# 开启 DHT11 Telemetry
# 设置 DHT11_GPIO_NUM = 5
idf.py build flash monitor
```

串口日志应看到:
```
Dht11Telemetry: DHT11 reading: temperature=28 C humidity=65%
```

> 里程碑:设备能周期读取温湿度,串口可见。

---

## 第 3 步:MQTT 遥测上报(约 3 小时)

### 3.1 拓展采集服务:加入 MQTT 客户端

在 Dht11TelemetryService 中新增:

```cpp
// 在 TaskLoop 中,读取成功后:
#if CONFIG_DHT11_ENABLE_MQTT_UPLOAD
if (EnsureMqttConnected()) {
    PublishReading(temperature, humidity);
}
#endif
```

### 3.2 MQTT 连接实现

```cpp
bool Dht11TelemetryService::EnsureMqttConnected() {
    // 1. 等待网络就绪
    if (Application::GetInstance().GetDeviceState() < kDeviceStateIdle) {
        return false;  // 网络未就绪,跳过
    }
    // 2. 读取配置(endpoint / client_id / username / password)
    // 3. 创建 MQTT 客户端
    // 4. 连接 broker
    // 5. 订阅命令主题(可选)
    return mqtt_->Connect(broker_address, broker_port, client_id, username, password);
}
```

### 3.3 发布 JSON 遥测

```cpp
bool Dht11TelemetryService::PublishReading(int temperature, int humidity) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", Board::GetInstance().GetUuid().c_str());
    cJSON_AddStringToObject(root, "sensor", "dht11");
    cJSON_AddNumberToObject(root, "temperature_c", temperature);
    cJSON_AddNumberToObject(root, "humidity_pct", humidity);
    cJSON_AddNumberToObject(root, "uptime_ms", esp_timer_get_time() / 1000);

    char* json_str = cJSON_PrintUnformatted(root);
    std::string payload(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);

    return mqtt_->Publish("xiaozhi/telemetry/dht11", payload);
}
```

### 3.4 订阅环境命令主题

```cpp
// 在 OnConnected 回调中:
mqtt_->Subscribe("xiaozhi/command/env");

// 在 OnMessage 回调中:
if (topic == "xiaozhi/command/env") {
    HandleEnvCommand(payload);
}
```

### 3.5 命令处理

```cpp
void HandleEnvCommand(const std::string& payload) {
    cJSON* root = cJSON_Parse(payload.c_str());
    std::string action = cJSON_GetObjectItem(root, "action")->valuestring;
    std::string message = cJSON_GetObjectItem(root, "message")->valuestring;

    if (action == "turn_on_fan") {
        Application::GetInstance().SetEnvironmentFanPower(true, "Cloud");
    } else if (action == "turn_off_fan") {
        Application::GetInstance().SetEnvironmentFanPower(false, "Cloud");
    }

    // 显示通知(需切到 UI 线程)
    if (!message.empty()) {
        Application::GetInstance().Schedule([message]() {
            Board::GetInstance().GetDisplay()->ShowNotification(message.c_str(), 4000);
        });
    }
    cJSON_Delete(root);
}
```

### 3.6 验证

用 MQTTX 连接 broker.emqx.io:1883,订阅 xiaozhi/telemetry/dht11,应收到:
```json
{"device_id":"esp32p4-demo","sensor":"dht11","temperature_c":28,"humidity_pct":65,"uptime_ms":182350}
```

发布到 xiaozhi/command/env:
```json
{"action":"turn_on_fan","message":"测试:开启风扇"}
```
设备屏幕应显示通知。

> 里程碑:温湿度数据通过 MQTT 上云,可远程下发指令。

---

## 第 4 步:风扇硬件驱动(约 3 小时)

### 4.1 硬件接线

```
L9110S          ESP32-P4
VCC       →     5V(或外部电源)
GND       →     GND
INA       →     GPIO6(PWM 调速)
INB       →     GPIO4(方向)
电机输出  →     直流风扇
```

### 4.2 修改板级配置

main/boards/esp-p4-function-ev-board/config.h:
```c
// Fan control via L9110S motor driver
#define FAN_INA_GPIO              GPIO_NUM_6
#define FAN_INB_GPIO              GPIO_NUM_4
#define FAN_PWM_FREQ_HZ           20000
#define FAN_LEDC_TIMER            LEDC_TIMER_3
#define FAN_LEDC_CHANNEL          LEDC_CHANNEL_3
#define FAN_PWM_DUTY_RESOLUTION   LEDC_TIMER_10_BIT
#define FAN_SPEED_LOW_DUTY_PERCENT    30
#define FAN_SPEED_MEDIUM_DUTY_PERCENT 60
#define FAN_SPEED_HIGH_DUTY_PERCENT   100
```

### 4.3 实现风扇初始化与驱动

esp-p4-function-ev-board.cc 中:

**初始化 GPIO 和 LEDC:**
```cpp
void InitializeFanGpio() {
    // INB:普通 GPIO 输出
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << FAN_INB_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);

    // INA:LEDC PWM 输出
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = FAN_PWM_DUTY_RESOLUTION,  // 10-bit
        .timer_num = FAN_LEDC_TIMER,
        .freq_hz = FAN_PWM_FREQ_HZ,                   // 20kHz
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = FAN_INA_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = FAN_LEDC_CHANNEL,
        .timer_sel = FAN_LEDC_TIMER,
        .duty = 0,  // 初始关闭
    };
    ledc_channel_config(&channel);

    gpio_set_level(FAN_INB_GPIO, 0);  // 拉低,不反转
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL);
}
```

**三档调速:**
```cpp
void ApplyFanSpeedLevel(int level) {
    // level: 0=关, 1=低, 2=中, 3=高
    level = level < 0 ? 0 : (level > 3 ? 3 : level);
    int duty_pct = 0;
    switch (level) {
        case 1: duty_pct = FAN_SPEED_LOW_DUTY_PERCENT; break;     // 30%
        case 2: duty_pct = FAN_SPEED_MEDIUM_DUTY_PERCENT; break;  // 60%
        case 3: duty_pct = FAN_SPEED_HIGH_DUTY_PERCENT; break;    // 100%
    }
    uint32_t max_duty = (1U << FAN_PWM_DUTY_RESOLUTION) - 1U;
    uint32_t duty = max_duty * duty_pct / 100;

    gpio_set_level(FAN_INB_GPIO, 0);  // 始终拉低
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL);

    fan_power_ = level > 0;
    fan_speed_level_ = level;
}
```

### 4.4 在 Board 基类中暴露风扇接口

main/boards/common/board.h 中添加虚函数:
```cpp
virtual bool HasFanControl() const { return false; }
virtual bool GetFanPower(bool& power) const { return false; }
virtual bool SetFanPower(bool power) { return false; }
```

### 4.5 在 Application 中封装

main/application.h:
```cpp
bool HasEnvironmentFanControl() const;
bool GetEnvironmentFanPower(bool& power) const;
bool SetEnvironmentFanPower(bool power, const std::string& source, bool notify = true);
```

main/application.cc:
```cpp
bool Application::SetEnvironmentFanPower(bool power, const std::string& source, bool notify) {
    auto& board = Board::GetInstance();
    bool has_fan = board.SetFanPower(power);

    if (notify) {
        Schedule([power, source]() {
            auto* display = Board::GetInstance().GetDisplay();
            if (display) {
                display->ShowNotification(power ? "风扇已开启" : "风扇已关闭", 3000);
                display->SetChatMessage("system", power ? "风扇已开启" : "风扇已关闭");
            }
        });
    }
    return has_fan;
}
```

### 4.6 验证

在 MQTTX 中发布 {"action":"turn_on_fan"} 到 xiaozhi/command/env,风扇应转动,屏幕显示"风扇已开启"。

> 里程碑:风扇可远程控制,屏幕有反馈。

---

## 第 5 步:MCP 环境工具注册(约 3 小时)

### 5.1 注册融合状态工具

在 esp-p4-function-ev-board.cc 的 InitializeTools() 中:

```cpp
mcp_server.AddTool("self.env.get_fusion_status",
    "获取设备当前融合环境状态,包括温度、湿度、摄像头可用性和风扇状态。",
    PropertyList(),  // 无参数
    [this](const PropertyList&) -> ReturnValue {
        auto& app = Application::GetInstance();

        int temperature = 0, humidity = 0;
        int64_t uptime = 0;
        bool has_dht11 = app.GetLatestDht11Reading(temperature, humidity, uptime);

        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "device_id", GetUuid().c_str());
        cJSON_AddStringToObject(root, "board_type", GetBoardType().c_str());
        cJSON_AddBoolToObject(root, "camera_available", camera_ != nullptr);
        cJSON_AddBoolToObject(root, "has_temperature_humidity", has_dht11);
        cJSON_AddBoolToObject(root, "fan_hardware_present", HasFanHardware());
        cJSON_AddBoolToObject(root, "fan_power", fan_power_);

        if (has_dht11) {
            cJSON_AddNumberToObject(root, "temperature_c", temperature);
            cJSON_AddNumberToObject(root, "humidity_pct", humidity);
            cJSON_AddNumberToObject(root, "uptime_ms", static_cast<double>(uptime));
        } else {
            cJSON_AddNullToObject(root, "temperature_c");
            cJSON_AddNullToObject(root, "humidity_pct");
            cJSON_AddNullToObject(root, "uptime_ms");
        }
        return root;
    });
```

### 5.2 注册动作执行工具

```cpp
mcp_server.AddTool("self.env.execute_action",
    "执行云端决策结果,在设备上产生本地反馈。",
    PropertyList({
        Property("action", kPropertyTypeString),   // turn_on_fan / turn_off_fan / notify
        Property("message", kPropertyTypeString, std::string("")),
    }),
    [this](const PropertyList& properties) -> ReturnValue {
        auto action = ToLower(properties["action"].value<std::string>());
        auto message = properties["message"].value<std::string>();

        // 白名单 + 归一化
        if (action == "turn_on_fan") {
            ApplyFanSpeedLevel(2);  // 默认中档
        } else if (action == "turn_off_fan") {
            ApplyFanSpeedLevel(0);
        }

        // 默认提示文案
        if (message.empty()) {
            if (action == "turn_on_fan") message = "环境闷热,建议开启风扇";
            else if (action == "turn_off_fan") message = "环境状态正常";
            else message = "环境状态已更新";
        }

        // UI 反馈(切到 UI 线程)
        Application::GetInstance().Schedule([this, message]() {
            GetDisplay()->ShowNotification(message.c_str(), 4000);
            GetDisplay()->SetChatMessage("system", message.c_str());
        });

        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", true);
        cJSON_AddStringToObject(root, "action", action.c_str());
        cJSON_AddBoolToObject(root, "fan_power", fan_power_);
        return root;
    });
```

### 5.3 注册风扇专属工具

```cpp
// self.fan.get_state — 查询风扇状态
// self.fan.turn_on   — 开风扇
// self.fan.turn_off  — 关风扇
// self.fan.set_speed — 设置档位(1/2/3)
```
(实现方式与上面类似,略)

### 5.4 验证

通过 MCP 协议调用 tools/list 应看到新增工具,调用 tools/call self.env.get_fusion_status 应返回融合 JSON。

> 里程碑:设备能力通过 MCP 暴露给大模型,云端可查询和下发动作。

---

## 第 6 步:接入云端大模型决策(约 3 小时)

### 6.1 配置小智云端服务

在 menuconfig 或 NVS 设置中配置:
- MQTT broker 地址
- 小智云端服务 URL
- 设备激活 token

### 6.2 云端提示词设计

在云端服务配置中加入系统提示词:

```text
你是一个云端环境感知决策助手,负责分析 ESP32-P4 设备上传的环境状态。

决策规则:
1. 如果 has_temperature_humidity=false,只提示"数据暂不可用"
2. 如果 temperature_c >= 30 且 humidity_pct >= 70,判定为"闷热",调用 self.env.execute_action 开风扇
3. 如果 temperature_c >= 30 且 humidity_pct < 70,判定为"偏热",提示通风
4. 如果 humidity_pct >= 80 且 temperature_c < 30,判定为"潮湿",提示除湿
5. 如果 fan_hardware_present=false,不要说"风扇已开启",说"建议开启风扇"
6. 只能调用设备公开的 MCP 工具,不能虚构数据
```

### 6.3 端到端联调

1. 设备上电,确认 Wi-Fi 连接 → 设备激活 → 进入空闲
2. 确认串口日志有 DHT11 周期读数
3. 确认 MQTTX 能收到遥测 JSON
4. 按 BOOT 键触发语音:问"现在环境怎么样"
5. 观察云端大模型是否调用 get_fusion_status
6. 观察大模型是否根据温湿度调用 execute_action
7. 观察风扇是否动作、屏幕是否显示通知

> 里程碑:完整闭环打通——语音询问 → 云端决策 → 风扇动作 → 屏幕反馈。

---

## 第 7 步:扩展功能(选做,约 4 小时)

### 7.1 SD 卡本地 WAV 播放

添加 main/audio/wav_reader.cc,支持:
- 8/16 bit PCM WAV 解码
- 单/双声道(双声道混为单声道)
- 采样率重采样匹配输出

注册 MCP 工具 self.audio_speaker.play_wav_file,语音命令"播放 xxx"。

### 7.2 SD 卡媒体包播放

添加 main/media/media_package_player.cc,支持:
- 解析 meta.json(fps / frame_count / 音频/帧目录)
- 按 FPS 定时显示 JPG 帧到 LVGL 预览区
- 音轨与帧同步播放

### 7.3 摄像头接入

BSP 摄像头初始化后,将 camera_available 纳入融合状态,供后续视觉扩展。

---

## 第 8 步:稳定性加固(约 3 小时)

### 8.1 任务创建失败处理

```cpp
void Dht11TelemetryService::Start() {
    if (started_.load()) return;
    started_.store(true);
    if (xTaskCreate(TaskEntry, "dht11_telemetry", 4096, this, 3, &task_handle_) != pdPASS) {
        started_.store(false);   // 回滚状态
        task_handle_ = nullptr;
        ESP_LOGE(TAG, "Failed to create DHT11 task");
    }
}
```

### 8.2 任务停止竞态保护

```cpp
void Dht11TelemetryService::Stop() {
    started_.store(false);
    // 通知任务从延时中醒来
    if (task_handle_ && xTaskGetCurrentTaskHandle() != task_handle_) {
        xTaskNotifyGive(task_handle_);
        // 等待任务退出
        for (int i = 0; i < 100 && task_handle_ != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    // 任务退出后才清理资源
    CleanupResources();
}
```

### 8.3 MQTT endpoint 健壮解析

不要用 std::stoi 直接拆 endpoint,实现健壮解析:
- 支持 host:port
- 支持 mqtts://host:port
- 支持 IPv6 方括号 [::1]:8883
- 非法格式返回错误,不抛异常

### 8.4 SD 卡写入防护

write_file_chunk 工具新增 overwrite 参数,默认 false,已存在文件必须显式授权才能覆盖。

### 8.5 媒体播放引用计数

WAV 播放任务创建失败时,回滚 local_media_playback_ref_count_,防止 TTS 被永久抑制;媒体包播放器用独立会话标记,停止时只释放自己的引用。

---

## 总结:功能实现顺序一览

| 步骤 | 功能 | 预计耗时 | 可否独立验证 |
| --- | --- | --- | --- |
| 0 | ESP-IDF 环境搭建 | 2h | idf.py build 通过 |
| 1 | 基础框架运行 | 3h | 屏幕显示 + 语音对话 |
| 2 | DHT11 温湿度采集 | 4h | 串口日志可见读数 |
| 3 | MQTT 遥测上报 + 命令订阅 | 3h | MQTTX 收发数据 |
| 4 | 风扇硬件驱动 | 3h | 风扇随指令转动 |
| 5 | MCP 环境工具注册 | 3h | tools/call 返回融合 JSON |
| 6 | 云端大模型联调 | 3h | 语音问"环境"→风扇动作 |
| 7 | 扩展功能(可选) | 4h | 本地媒体播放 |
| 8 | 稳定性加固 | 3h | 异常场景不崩溃 |
| **合计** | | **约 28h** | |

---

## 附录:常见问题排查

| 问题 | 可能原因 | 排查方法 |
| --- | --- | --- |
| DHT11 读不到数据 | 时序被中断破坏 | 检查 taskENTER_CRITICAL 是否覆盖读取全过程 |
| DHT11 偶尔读到 100/100 | 校验和失败 | 用 dht11_get_last_error() 看具体错误码 |
| MQTT 连接不上 | 网络未就绪 | 检查 GetDeviceState() >= kDeviceStateIdle |
| 风扇不转 | INB 未拉低 | 用万用表测 GPIO4 是否为低电平 |
| MCP 工具调用无响应 | 工具未注册 | 用 tools/list 确认工具列表 |
| 屏幕通知不显示 | 未切到 UI 线程 | 检查是否用了 Application::Schedule() |
| 采集任务崩溃 | endpoint 解析异常 | 确认 URL 格式被健壮解析(不用 std::stoi) |
