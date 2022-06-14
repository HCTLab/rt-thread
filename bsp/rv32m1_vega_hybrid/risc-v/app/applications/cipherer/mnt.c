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
    int error;

    disk_initialize(0);  // Just to avoid 'elm_init()' function being removed by linker (nobody calls it)
    error = dfs_mount("sd0", "/", "elm", 0, 0);

    return error;
}
//INIT_ENV_EXPORT(mnt_init);  // Manually mounted from main()
#endif
