#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "sqlite3.h"
#include "db_manager.h"

static sqlite3 *db = NULL;
static char current_db_name[128];

// 初始化数据库
int db_init(const char *db_name) {
    // 保存文件名
    strncpy(current_db_name, db_name, sizeof(current_db_name));
    
    // 1. 打开数据库
    int rc = sqlite3_open(db_name, &db);
    if (rc) {
        fprintf(stderr, "[DB] Can't open database: %s\n", sqlite3_errmsg(db));
        return -1;
    }

    // 2. [重要] 先设置忙碌超时 (放在最前面，防止后续操作被锁)
    sqlite3_busy_timeout(db, 5000);

    // 3. 开启高性能模式 (WAL + Normal)
    sqlite3_exec(db, "PRAGMA journal_mode=WAL;", 0, 0, 0);
    sqlite3_exec(db, "PRAGMA synchronous=NORMAL;", 0, 0, 0);

    char *err_msg = 0;
    
    // --- 定义 SQL 语句 ---

    // 表1: 节点注册表
    const char *sql_nodes = 
        "CREATE TABLE IF NOT EXISTS node_registry ("
        "uid INTEGER PRIMARY KEY, "
        "node_id INTEGER, "
        "join_time DATETIME DEFAULT CURRENT_TIMESTAMP);";

    // 表2: 传感器数据表
    const char *sql_data = 
        "CREATE TABLE IF NOT EXISTS sensor_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "node_id INTEGER, "
        "temperature REAL, "
        "seq_num INTEGER, "
        "rssi INTEGER, "
        "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP);";

    // 表3: 节点最新状态表 (快照)
    const char *sql_status = 
        "CREATE TABLE IF NOT EXISTS node_status ("
        "node_id INTEGER PRIMARY KEY, "
        "last_temp REAL, "
        "last_rssi INTEGER, "
        "last_seen DATETIME DEFAULT CURRENT_TIMESTAMP);";

    // 索引1: 加速按节点查找
    const char *idx1 = "CREATE INDEX IF NOT EXISTS idx_node_id ON sensor_data(node_id);";
    // 索引2: 加速按时间查找
    const char *idx2 = "CREATE INDEX IF NOT EXISTS idx_timestamp ON sensor_data(timestamp);";
    // 索引3: 复合索引
    const char *idx3 = "CREATE INDEX IF NOT EXISTS idx_node_time ON sensor_data(node_id, timestamp);";

    // --- [修正顺序] 先建表，再建索引 ---

    // 4. 执行建表
    sqlite3_exec(db, sql_nodes, 0, 0, &err_msg);
    sqlite3_exec(db, sql_data, 0, 0, &err_msg);
    sqlite3_exec(db, sql_status, 0, 0, &err_msg);

    // 5. 执行建索引 (表存在了才能建索引)
    sqlite3_exec(db, idx1, 0, 0, 0);
    sqlite3_exec(db, idx2, 0, 0, 0);
    sqlite3_exec(db, idx3, 0, 0, 0);

    return 0;
}

// 获取或分配 Node ID
int db_get_or_create_node_id(uint32_t uid) {
    if (!db) return -1;
    
    sqlite3_stmt *stmt;
    int node_id = -1;

    // 1. 查询是否存在
    const char *query = "SELECT node_id FROM node_registry WHERE uid = ?;";
    sqlite3_prepare_v2(db, query, -1, &stmt, 0);
    sqlite3_bind_int64(stmt, 1, uid);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        node_id = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);

    // 2. 如果存在，直接返回
    if (node_id > 0) return node_id;

    // 3. 如果不存在，分配新的 (Max ID + 1)
    // 简单起见，我们查一下当前最大的 ID
    int max_id = 0;
    const char *query_max = "SELECT MAX(node_id) FROM node_registry;";
    sqlite3_prepare_v2(db, query_max, -1, &stmt, 0);
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        max_id = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
    
    node_id = max_id + 1;

    // 4. 插入新记录
    const char *insert = "INSERT INTO node_registry (uid, node_id) VALUES (?, ?);";
    sqlite3_prepare_v2(db, insert, -1, &stmt, 0);
    sqlite3_bind_int64(stmt, 1, uid);
    sqlite3_bind_int(stmt, 2, node_id);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    printf("[DB] New Node Registered! UID: 0x%08X -> ID: %d\n", uid, node_id);
    return node_id;
}

// 插入数据 (现在只负责存，不负责开关文件)
int db_insert_data(int node_id, float temp, int seq_num, int rssi) {
    if (!db) return -1;

    // 插入历史数据
    const char *sql = "INSERT INTO sensor_data (node_id, temperature, seq_num, rssi) VALUES (?, ?, ?, ?);";
    sqlite3_stmt *stmt;
    sqlite3_prepare_v2(db, sql, -1, &stmt, 0);
    
    sqlite3_bind_int(stmt, 1, node_id);
    sqlite3_bind_double(stmt, 2, temp);
    sqlite3_bind_int(stmt, 3, seq_num);
    sqlite3_bind_int(stmt, 4, rssi);
    
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    // 同时更新“最新状态表” (Insert or Replace)
    db_update_node_status(node_id, temp, rssi);

    return 0;
}

// 更新节点最新快照
int db_update_node_status(int node_id, float temp, int rssi) {
    const char *sql = 
        "INSERT INTO node_status (node_id, last_temp, last_rssi, last_seen) "
        "VALUES (?, ?, ?, CURRENT_TIMESTAMP) "
        "ON CONFLICT(node_id) DO UPDATE SET "
        "last_temp=excluded.last_temp, "
        "last_rssi=excluded.last_rssi, "
        "last_seen=CURRENT_TIMESTAMP;";
        
    sqlite3_stmt *stmt;
    sqlite3_prepare_v2(db, sql, -1, &stmt, 0);
    sqlite3_bind_int(stmt, 1, node_id);
    sqlite3_bind_double(stmt, 2, temp);
    sqlite3_bind_int(stmt, 3, rssi);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return 0;
}

// 开启事务 (批量写入加速)
int db_begin_transaction(void) {
    if (!db) return -1;
    return sqlite3_exec(db, "BEGIN TRANSACTION;", 0, 0, 0);
}

// 提交事务
int db_commit_transaction(void) {
    if (!db) return -1;
    return sqlite3_exec(db, "COMMIT;", 0, 0, 0);
}

// 清理旧数据 (保留最近 N 天)
int db_prune_old_data(int keep_days) {
    if (!db) return -1;
    
    char sql[256];
    // SQLite 的时间函数：date('now', '-7 days')
    snprintf(sql, sizeof(sql), 
             "DELETE FROM sensor_data WHERE timestamp < datetime('now', '-%d days');", 
             keep_days);
             
    int rc = sqlite3_exec(db, sql, 0, 0, 0);
    if (rc == SQLITE_OK) {
        // printf("[DB] Pruned data older than %d days.\n", keep_days);
        // 执行 VACUUM 释放磁盘空间 (耗时操作，偶尔做)
        // sqlite3_exec(db, "VACUUM;", 0, 0, 0); 
    }
    return rc;
}

void db_close(void) {
    if (db) {
        sqlite3_close(db);
        db = NULL;
    }
}