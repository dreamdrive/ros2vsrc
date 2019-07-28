// vsrc_control2.cpp
// joy_nodeからジョイスティックのトピック(/joy)をsubscribeし、VS-RC003用のコマンドに変換し、vsrc_connect2ノードに対してPublishするノード
// 注意:vsrc_controlとは互換性はありません。
//
// 接続するコントローラーは、XBOX360コントローラーを想定しています
//
// MIT License
// Copyright (c) 2019 Hirokazu Onomichi

#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "ros2vsrc/VsrcControl.h"

bool w_torque = 0;	// トルクON/OFF(書き込み)

bool b_left = 0;
bool b_down = 0;
bool b_right = 0;
bool b_up = 0;
bool b_start = 0;
bool b_R3 = 0;
bool b_L3 = 0;
bool b_select = 0;
bool b_square = 0;
bool b_cross = 0;
bool b_circle = 0;
bool b_triangle = 0;
bool b_R1 = 0;
bool b_L1 = 0;
bool b_R2 = 0;
bool b_L2 = 0;
short s_lx = 0;
short s_ly = 0;
short s_rx = 0;
short s_ry = 0;
bool b_shiitake360 = 0;

void joy_callback360(const sensor_msgs::Joy& joy_msg){

	float axes1[8];
	int axes2[8];

	axes1[0]=joy_msg.axes[0];	// 左スティック横方向(XBOX360)	左:1.0~右-1.0
	axes1[1]=joy_msg.axes[1];	// 左スティック縦方向(XBOX360)	上:1.0~下-1.0
	axes1[2]=joy_msg.axes[2];	// 右スティック横方向(XBOX360)	左:1.0~右-1.0
	axes1[3]=joy_msg.axes[3];	// 右スティック縦方向(XBOX360)	上:1.0~下-1.0
	s_lx = (short)(127.0 * axes1[0]);
	s_ly = (short)(127.0 * axes1[1]);
	s_rx = (short)(127.0 * axes1[2]);
	s_ry = (short)(127.0 * axes1[3]);
	
	axes2[4]=joy_msg.axes[4];	// RTボタンアナログ値(XBOX360)	開放：1 押し込み:-1
	axes2[5]=joy_msg.axes[5];	// LTボタンアナログ値(XBOX360)	開放：1 押し込み:-1
	axes2[6]=joy_msg.axes[6];	// 十字キー(横)(XBOX360)	左:1 右:-1 押さない:0
	axes2[7]=joy_msg.axes[7];	// 十字キー(縦)(XBOX360)	上:1 下:-1 押さない:0

	switch (axes2[6]) {
		case 1:			// 左の時
			b_left = 1;		// ← : 1
			b_right = 0;		// → : 0
			break;
		case -1:		// 右の時
			b_left = 0;		// ← : 0
			b_right = 1;		// → : 1
			break;
		default:		// 無状態
			b_left = 0;		// ← : 0
			b_right = 0;		// → : 0
			break;
	}

	switch (axes2[7]) {
		case 1:			// 上の時
			b_down = 0;		// ↓ : 0
			b_up = 1;		// ↑ : 1
			break;
		case -1:		// 下の時
			b_down = 1;		// ↓ : 1
			b_up = 0;		// ↑ : 0
			break;
		default:		// 無状態
			b_down = 0;		// ↓ : 0
			b_up = 0;		// ↑ : 0
			break;
	}

	if (axes2[4] == -1){	//RTボタン → R2ボタンに変換
		b_R2 = 1;
	}else{
		b_R2 = 0;
	}

	if (axes2[5] == -1){	//LTボタン → L2ボタンに変換
		b_L2 = 1;
	}else{
		b_L2 = 0;
	}

	b_cross = joy_msg.buttons[0];	// A(XBOX360)
	b_circle = joy_msg.buttons[1];	// B(XBOX360)
	b_square = joy_msg.buttons[2];	// X(XBOX360)
	b_triangle = joy_msg.buttons[3];	// Y(XBOX360)
	b_L1 = joy_msg.buttons[4];	// LB(XBOX360)
	b_R1 = joy_msg.buttons[5];	// RB(XBOX360)
	b_select = joy_msg.buttons[6];	// BACK(XBOX360)
	b_start = joy_msg.buttons[7];	// START(XBOX360)
	b_L3 = joy_msg.buttons[9];	// 左スティック押し込み(XBOX360)
	b_R3 = joy_msg.buttons[10];	// 右スティック押し込み(XBOX360)

	b_shiitake360 = joy_msg.buttons[8];	// しいたけボタン(XBOX360)
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vsrc_control2");

	ros::NodeHandle n;
	ros2vsrc::VsrcControl vsrcctl;
	int f_shiitake_b = 0;

	//publish
	ros::Publisher vsrc_pub = n.advertise<ros2vsrc::VsrcControl>("vsrc_control2",10);

	//subscriibe
	ros::Subscriber joy_sub	= n.subscribe("joy", 10, joy_callback360);

	ros::Rate loop_rate(30);

	while (ros::ok()){

		if (b_shiitake360){	// しいたけボタンが押された時
			if (f_shiitake_b == 0){		// しいたけボタンがはじめておされたとき
				w_torque = 1 - w_torque;	// w_torqueを反転（ONならOFFに、OFFならONに）
				f_shiitake_b = 1;			// 継続フラグを立てる
			}
		}else{
			f_shiitake_b = 0;
		}

		vsrcctl.w_torque =	w_torque;
		vsrcctl.b_left = b_left;
		vsrcctl.b_down = b_down;
		vsrcctl.b_right = b_right;
		vsrcctl.b_up = b_up;
		vsrcctl.b_start = b_start;
		vsrcctl.b_R3 = b_R3;
		vsrcctl.b_L3 = b_L3;
		vsrcctl.b_select = b_select;
		vsrcctl.b_square = b_square;
		vsrcctl.b_cross = b_cross;
		vsrcctl.b_circle = b_circle;
		vsrcctl.b_triangle = b_triangle;
		vsrcctl.b_R1 = b_R1;
		vsrcctl.b_L1 = b_L1;
		vsrcctl.b_R2 = b_R2;
		vsrcctl.b_L2 = b_L2;
		vsrcctl.s_lx = s_lx;
		vsrcctl.s_ly = s_ly;
		vsrcctl.s_rx = s_rx;
		vsrcctl.s_ry = s_ry;

		vsrc_pub.publish(vsrcctl);			// 値を送信

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
