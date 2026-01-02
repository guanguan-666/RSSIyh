#ifndef _DB_MANAGER_H_
#define _DB_MANAGER_H_

#include <stdint.h>

// 初始化数据库
int db_init(const char *db_name);

// 获取或创建节点 ID
int db_get_or_create_node_id(uint32_t uid);

// 插入传感器数据
int db_insert_data(int node_id, float temp, int seq_num, int rssi);

// === [新增] 事务控制 (用于 TDMA 批量写入) ===
int db_begin_transaction(void);
int db_commit_transaction(void);

// === [新增] 数据老化 (清理旧数据) ===
// keep_days: 保留最近几天的数据
int db_prune_old_data(int keep_days);

// === [新增] 更新节点最新状态 (快照) ===
int db_update_node_status(int node_id, float temp, int rssi);

// 关闭数据库
void db_close(void);

#endif