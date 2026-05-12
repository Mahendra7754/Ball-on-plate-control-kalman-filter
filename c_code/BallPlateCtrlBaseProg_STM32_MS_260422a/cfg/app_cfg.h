/*
*********************************************************************************************************
*                       Application Specific Configuration Module
*********************************************************************************************************
*/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

//#define  uC_PROBE_OS_PLUGIN              DEF_ENABLED            /* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */
//#define  uC_PROBE_COM_MODULE             DEF_ENABLED

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO                  25
#define  APP_TASK_KBD_PRIO                    26
#define  APP_TASK_PROBE_STR_PRIO              27

#define  OS_PROBE_TASK_PRIO                   28
#define  OS_PROBE_TASK_ID                     28

#define  OS_TASK_TMR_PRIO              (OS_LOWEST_PRIO - 2)

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                            Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/

#define  APP_TASK_START_STK_SIZE             256
#define  APP_TASK_KBD_STK_SIZE               256
#define  APP_TASK_PROBE_STR_STK_SIZE         512

#define  OS_PROBE_TASK_STK_SIZE              512

/*
*********************************************************************************************************
*                               uC/Probe plug-in for uC/OS-II CONFIGURATION
*********************************************************************************************************
*/

//#define  OS_PROBE_TASK                         0                /* Task will be created for uC/Probe OS Plug-In             */
//#define  OS_PROBE_TMR_32_BITS                  0                /* uC/Probe OS Plug-In timer is a 32-bit timer              */
//#define  OS_PROBE_HOOKS_EN                     0                /* Hooks to update OS_TCB profiling members will be included*/

/*
*********************************************************************************************************
*                                      uC/OS-II DCC CONFIGURATION
*********************************************************************************************************
*/

#define  OS_CPU_ARM_DCC_EN                     0


/*
*********************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#define  TRACE_LEVEL_OFF                       0
#define  TRACE_LEVEL_INFO                      1
#define  TRACE_LEVEL_DEBUG                     2

#define  APP_TRACE_LEVEL                TRACE_LEVEL_DEBUG
#define  APP_TRACE                        

#define  APP_TRACE_INFO(x)            ((APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_TRACE x) : (void)0)
#define  APP_TRACE_DEBUG(x)           ((APP_TRACE_LEVEL >= TRACE_LEVEL_DEBUG) ? (void)(APP_TRACE x) : (void)0)

#endif
