#ifndef RP__OFFLINE__JUDGE
#define RP__OFFLINE__JUDGE

#include "sys.h"

typedef enum
{
	WORKING,								//WORKING    		对应的是工作态，这种情况下可以直接操作
	REFRESHING,							//REFRESHING	 	对应的是刷新态，这种情况下一般不允许直接操作
	DISCONNECTING,					//DISCONNECTING 对应的是离线态，这种情况下需要做一些应急措施
	RECONNECTING						//RECONNECTING  重新链接缓冲态
}Module_STATE;

typedef struct judge_function
{
	u8 (*deal_working)(void);
	u8 (*deal_refreshing)(void);
	u8 (*deal_reconnecting)(void);
	u8 (*deal_disconnecting)(void);
	
}judge_function;

typedef struct OffLine_Judge_Module
{
	Module_STATE state;
	u16 cycle;
	u16 cycle_cnt;
	u16 reconnect_cnt;
	
	judge_function func;
	
}OffLine_Judge_Module;

//这个函数应该放到1ms中断里面
//每1ms对当前状态进行判断
void Judge_State_Flash(OffLine_Judge_Module *pJudge);

//初始化判断结构体
void Judge_Init(OffLine_Judge_Module *pJudge);

//判断结构体设置参数
void Judge_Set(OffLine_Judge_Module *pJudge, u16 cycle);

//重新连接后调用这个改变状态
void Judge_State_ReConnect(OffLine_Judge_Module *pJudge);

//空函数，无作用
u8 Judge_Func_Void(void);

#endif

