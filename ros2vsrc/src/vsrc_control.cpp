// vsrc_control.cpp
// joy_nodeからジョイスティックのトピック(/joy)をsubscribeし、VS-RC003用のコマンドに変換し、vsrc_connectノードに対してPublishするノード
//
// 接続するコントローラーは、XBOX360コントローラーを想定しています
//
// MIT License
// Copyright (c) 2019 Hirokazu Onomichi


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

#define	BUFF_SZ	(256)

int but1[16];
int but2[6];
float axes1[8];
int axes2[8];
int sv_power = 0;
int f_shiitake_b = 0;

void joy_callback(const sensor_msgs::Joy& joy_msg){

  	axes1[0]=joy_msg.axes[0];	// 左スティック横方向(XBOX360)	左:1.0?右-1.0
	axes1[1]=joy_msg.axes[1];	// 左スティック縦方向(XBOX360)	上:1.0?下-1.0
	axes1[2]=joy_msg.axes[2];	// 右スティック横方向(XBOX360)	左:1.0?右-1.0
	axes1[3]=joy_msg.axes[3];	// 右スティック縦方向(XBOX360)	上:1.0?下-1.0
	axes1[4]=joy_msg.axes[4];	// RTボタンアナログ値(XBOX360)	開放：1.0?押し込み:-1.0
	axes1[5]=joy_msg.axes[5];	// LTボタンアナログ値(XBOX360)	開放：1.0?押し込み:-1.0
	axes1[6]=joy_msg.axes[6];	// 十字キー(横)(XBOX360)	左:1 右:-1 押さない:0
	axes1[7]=joy_msg.axes[7];	// 十字キー(縦)(XBOX360)	上:1 下:-1 押さない:0

	axes2[4]=joy_msg.axes[4];	// RTボタンアナログ値(XBOX360)	開放：1 押し込み:-1
	axes2[5]=joy_msg.axes[5];	// LTボタンアナログ値(XBOX360)	開放：1 押し込み:-1
	axes2[6]=joy_msg.axes[6];	// 十字キー(横)(XBOX360)	左:1 右:-1 押さない:0
	axes2[7]=joy_msg.axes[7];	// 十字キー(縦)(XBOX360)	上:1 下:-1 押さない:0

	but1[0] = joy_msg.buttons[0];	// A(XBOX360)
	but1[1] = joy_msg.buttons[1];	// B(XBOX360)
	but1[2] = joy_msg.buttons[2];	// X(XBOX360)
	but1[3] = joy_msg.buttons[3];	// Y(XBOX360)
	but1[4] = joy_msg.buttons[4];	// LB(XBOX360)
	but1[5] = joy_msg.buttons[5];	// RB(XBOX360)
	but1[6] = joy_msg.buttons[6];	// BACK(XBOX360)
	but1[7] = joy_msg.buttons[7];	// START(XBOX360)
	but1[8] = joy_msg.buttons[8];	// しいたけボタン(XBOX360)
	but1[9] = joy_msg.buttons[9];	// 左スティック押し込み(XBOX360)
	but1[10] = joy_msg.buttons[10];	// 右スティック押し込み(XBOX360)
	but1[11] = joy_msg.buttons[11];	// NC
	but1[12] = joy_msg.buttons[12];	// NC
	but1[13] = joy_msg.buttons[13];	// NC
	but1[14] = joy_msg.buttons[14];	// NC
	but1[15] = joy_msg.buttons[15];	// NC

	switch (axes2[6]) {
		case 1:			// 左の時
			but2[0] = 1;		// ← : 1
			but2[2] = 0;		// → : 0
			break;
		case -1:		// 右の時
			but2[0] = 0;		// ← : 0
			but2[2] = 1;		// → : 1
			break;
		default:		// 無状態
			but2[0] = 0;		// ← : 0
			but2[2] = 0;		// → : 0
			break;
	}

	switch (axes2[7]) {
		case 1:			// 上の時
			but2[1] = 0;		// ↓ : 0
			but2[3] = 1;		// ↑ : 1
			break;
		case -1:		// 下の時
			but2[1] = 1;		// ↓ : 1
			but2[3] = 0;		// ↑ : 0
			break;
		default:		// 無状態
			but2[1] = 0;		// ↓ : 0
			but2[3] = 0;		// ↑ : 0
			break;
	}

	if (axes2[4] == -1){	//RTボタン → R2ボタンに変換
		but2[4] = 1;
	}else{
		but2[4] = 0;
	}

	if (axes2[5] == -1){	//LTボタン → L2ボタンに変換
		but2[5] = 1;
	}else{
		but2[5] = 0;
	}

}

int main(int argc, char **argv){
	ros::init(argc, argv, "vsrc_control");
	ros::NodeHandle n;
	int tmp1,tmp2;
	short stk_rx,stk_ry,stk_lx,stk_ly;
	//char numStr[15];
	char rbuf[BUFF_SZ],wbuf[BUFF_SZ];

	//publish
	ros::Publisher vsrc_pub = n.advertise<std_msgs::String>("vsrc_write", 30);

	//subscriibe
	ros::Subscriber joy_sub	= n.subscribe("joy", 10, joy_callback);

	ros::Rate loop_rate(10);
	while (ros::ok()){
		std_msgs::String msg;
	
		//送受信メッセージバッファのクリア
		memset(wbuf,0,sizeof(wbuf));
		memset(rbuf,0,sizeof(rbuf));

		if (but1[8]){	// しいたけボタンが押された時
			
			if (f_shiitake_b==0){	// しいたけボタンがはじめておされたとき
				sv_power = 1 - sv_power;	// sv_powerを反転（ONならOFFに、OFFならONに）
			}

			sprintf(wbuf,"w 2009f6 %02x 00",sv_power);		

			f_shiitake_b = 1;	// 継続フラグを立てる

		}else{		// しいたけボタンが押されてない時

			tmp1 = but2[5] + but2[4] * 2 + but1[4] * 4 + but1[5] * 8 + but1[3] * 16 + but1[1] * 32+ but1[0] * 64 + but1[2] * 128;
			tmp2 = but1[6] + but1[9] * 2 + but1[10] * 4 + but1[7] * 8 + but2[3] * 16 + but2[2] * 32+ but2[1] * 64 + but2[0] * 128;

			stk_lx = (short)(127.0 * axes1[0]);		
			stk_ly = (short)(127.0 * axes1[1]);
			stk_rx = (short)(127.0 * axes1[3]);		
			stk_ry = (short)(127.0 * axes1[4]);

			//改行コードを含まないVS-RC003コマンド生成
			sprintf(wbuf,"w 2009e2 %02x %02x 00 00 00 00 %02x %02x %02x %02x %02x %02x %02x %02x", tmp1,tmp2,stk_rx&0x00ff,(stk_rx>>8)&0x00ff,stk_ry&0x00ff,(stk_ry>>8)&0x00ff,stk_lx&0x00ff,(stk_lx>>8)&0x00ff,stk_ly&0x00ff,(stk_ly>>8)&0x00ff);
			//                     ==(241)== -242- -243- ==(244)== ==(245)== ==(246)== ==(247)==    

			// test
			//printf("w 2009e2 %02x 00", tmp1);
			//printf("%02x %02x\n",stk_ly&0x00ff,(stk_ly>>8)&0x00ff);

			f_shiitake_b = 0;
		}

		msg.data = wbuf;

		ROS_INFO("%s", msg.data.c_str());
		vsrc_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
