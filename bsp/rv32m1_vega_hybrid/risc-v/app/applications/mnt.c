/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>

int mnt_init(void)
{
    int error = dfs_mount("sd0", "/", "elm", 0, 0);

    if (error == 0)  rt_kprintf("File System initialized!\n");
    else             rt_kprintf("File System initialized failed!\n");

    return error;
}
INIT_ENV_EXPORT(mnt_init);
#endif
