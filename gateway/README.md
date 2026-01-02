gateway_project/
├── libs_install/           # [已存在] 存放交叉编译好的第三方库 (SQLite, MQTT)
│   ├── include/            # .h 头文件
│   └── lib/                # .so 动态库文件
├── src/                    # [核心] 所有的源代码都在这里
│   ├── main.c              # 程序入口 (主循环、线程调度)
│   ├── platform/           # 硬件抽象层 (HAL) - 负责跟 Linux 底层打交道
│   │   ├── linux-hal.c     # 【核心】你刚才写的 SPI/GPIO 驱动适配代码
│   │   └── linux-hal.h     # 对应的头文件
│   ├── radio/              # 射频驱动层 (官方源码，基本不动)
│   │   ├── SX1278.c        # Semtech 官方逻辑代码
│   │   ├── SX1278.h
│   │   └── SX1278_hw.h     # 我们修改过的硬件定义头文件
│   ├── protocol/           # 协议层 (你的毕设核心 H-TDMA)
│   │   ├── htdma.c         # H-TDMA 组网逻辑 (信标、时隙分配)
│   │   └── htdma.h
│   ├── database/           # 数据存储层
│   │   ├── db_manager.c    # 封装 SQLite 操作 (存数据、查数据)
│   │   └── db_manager.h
│   └── mqtt/               # 云端通信层
│       ├── mqtt_client.c   # 封装 Paho MQTT 发送/接收
│       └── mqtt_client.h
├── Makefile                # 自动化编译脚本
└── README.md               # 项目说明



