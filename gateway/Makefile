# 1. 编译器设置
# 请确保你的环境变量中已经包含了交叉编译器，或者使用绝对路径
CC = arm-linux-gnueabihf-gcc

# 编译选项
# -I./src: 确保能引用到根目录下的 app_lora_gateway.h 和 app_lora_protocol.h
CFLAGS = -Wall -g \
         -I./libs_install/include \
         -I./src \
         -I./src/platform \
         -I./src/radio \
         -I./src/protocol \
         -I./src/database \
         -I./src/mqtt

# 2. 库文件设置 
# -static: 静态编译，方便移植到板子上直接运行
# 库链接顺序非常重要：依赖方在前，被依赖方在后
LDFLAGS = -static -L./libs_install/lib \
          -lpaho-mqtt3c -lsqlite3 -lpthread -ldl -lm

# 3. 源文件列表
# 新增了 src/app_lora_gateway.c
SRCS = src/main.c \
       src/app_lora_gateway.c \
       src/platform/linux-hal.c \
       src/radio/SX1278.c \
       src/database/db_manager.c

# 暂时不用的文件（请放在变量定义之外，避免续行符错误）
# src/mqtt/mqtt_client.c

# 4. 生成目标文件列表 (.c -> .o)
OBJS = $(SRCS:.c=.o)

# 5. 目标程序名
TARGET = gateway_app

# 6. 默认任务
all: $(TARGET)

# 链接
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	@echo "Build Success! -> $(TARGET)"

# 编译每个 .c 文件
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 清理
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean