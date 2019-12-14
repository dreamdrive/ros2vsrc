// vsrc_connect
// VS-RC003を制御するノード
//
// シリアルポートのデバイス名は "/dev/ttyUSB0" でシリアルポートを3.3V系の信号に変換し、VS-RC003のCN6もしくはCN7のTxD,RxD,GNDと接続して使用してください
// RaspberryPiの場合は、GPIOが3.3V系のためVS-RC003のCN6もしくはCN7のTxD,RxD,GNDと、それぞれ直結して使用する
// 
// シリアルポートの名称が変わる場合は、下記のようにプライベートパラメータにシリアルポートを指定して起動してください。(RaspberryPiの場合)
// $ rosrun ros2vsrc vsrc_connect _serialdev:=/dev/ttyAMA0
//
// MIT License
// Copyright (c) 2019 Hirokazu Onomichi

#include "ros/ros.h"

#include "sensor_msgs/BatteryState.h"
#include "ros2vsrc/VsrcState.h"
#include "ros2vsrc/VsrcControl.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define BUFF_SZ (256)				// 送受信バッファのサイズ
#define SERIAL_PORT "/dev/ttyUSB0"	// デフォルトのシリアルポート デバイス名

#define FALSE 0
#define TRUE 1
#define DEBUG FALSE

#define WAIT_TIME 20000	// (usec) シリアルポートに対するコンピュータの処理能力によって調整(raspiは0、core i7では大きめの値に要調整必要)

int fd1;			// シリアルポート

int r_voltage;		// 電圧
int r_mode;			// モード
int r_map;			// マップ
int r_ctltype;		// コントロールの種類(未接続=0、ゲームパッド=1、ProBo=2、RRC-T11=3)
int r_torque;		// トルクON/OFF(読み取り)
int r_idle;			// アイドル状態

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

int open_serial(const char *device_name){
	int fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(fd1, F_SETFL,0);
	//load configuration
	struct termios conf_tio;
	tcgetattr(fd1,&conf_tio);
	//set baudrate : VS-RC003 = ボーレート115200bps
	speed_t BAUDRATE = B115200;
	cfsetispeed(&conf_tio, BAUDRATE);
	cfsetospeed(&conf_tio, BAUDRATE);
	//non canonical, non echo back
	conf_tio.c_lflag &= ~( ECHO | ICANON );
	// inputの設定
	conf_tio.c_iflag &= IGNCR;	// 受信したIGNCR (CRLF)のCRを無視する(そのまま受信する) (重要)
	// outputの設定
	conf_tio.c_oflag &= ~ONLCR;	// 送信時LFをCRLFに変換する機能:ONLCRを反転(OFF)にする (重要)
	//non blocking
	conf_tio.c_cc[VMIN]=0;
	conf_tio.c_cc[VTIME]=0;
	//store configuration
	tcsetattr(fd1,TCSANOW,&conf_tio);
	return fd1;
}

////////////////////////////////////////////////////////////////
//
// debug_print
// 送受信したパケットを2行に表示（改行コードを'R'と'N'に置換)
//
////////////////////////////////////////////////////////////////
void debug_print(char *wbuf, char *rbuf){
	char wbuf_d[BUFF_SZ],rbuf_d[BUFF_SZ];
	int i;

	//表示用に整形
	for (i = 0; i<BUFF_SZ; i++) {
		wbuf_d[i] = wbuf[i];	// デバッグ用
		if (wbuf[i] == '\n') wbuf_d[i]='N';	// デバッグ用
		if (wbuf[i] == '\r') wbuf_d[i]='R';	// デバッグ用
	}

	//表示用に整形
	for (i = 0; i<BUFF_SZ; i++) {
		rbuf_d[i] = rbuf[i];	// デバッグ用
		if (rbuf[i] == '\n') rbuf_d[i]='N';	// デバッグ用
		if (rbuf[i] == '\r') rbuf_d[i]='R';	// デバッグ用
	}

	ROS_INFO("TX(%03d) >> %s",(int)strlen(wbuf),wbuf_d);
	ROS_INFO("RX(%03d) << %s",(int)strlen(rbuf),rbuf_d);
}

////////////////////////////////////////////////////////////////
//	sendmessage関数
//	メッセージを送受信する関数。
//	
//	・引数
//		char *wbuf	送信メッセージバッファへのポインタ
//		char *rbuf	受信メッセージバッファへのポインタ
////////////////////////////////////////////////////////////////

int sendmessage(char *wbuf, char *rbuf){

	bool werr(FALSE),rerr(FALSE);
	int i = 0, cr_num = 0 , flag = 0 , j = 0;

	if(!write(fd1,wbuf,(int)strlen(wbuf))) werr = TRUE;	// VS-RC003へ送信 / 書き込みができなければエラーフラグをたてる

	//メッセージの受信
	//メッセージの最後まで1byteずつ読み込みを行なう
	//メッセージの最後の判断は、2回目に改行('\n')が登場する場所。1回目は送信メッセージ、2回目はCPUボードが付記するもの
	//また、受信がタイムアウトを迎えた場合もループを抜ける
	
	usleep(WAIT_TIME);		// 送受信切替時のウェイト環境依存

	while (1) {	
		flag = read(fd1, &rbuf[i], 1);	// 1文字受信(返り値は受信バイト数(1):受信バッファが空なら(0) 
		if(flag > 0){	//受信した時
			if(rbuf[i] == 0x0a) cr_num++;	//その文字が改行コードなら改行カウントアップ
			i++; 							//次の文字へ
		}
		else{		//受信しなかったとき
			j++; 							// タイムアウト用のカウントアップ
		}

		if ((rbuf[0]=='w') && (cr_num >= 1)) break;	//wコマンドの時、改行コードが1つ来ると無限ループブレイク
		if ((rbuf[0]=='r') && (cr_num >= 2)) break;	//rコマンドの時、改行コードが2つ来ると無限ループブレイク

		if (j >= 10000) {			// 10000回繰り返すとタイムアウト終了
			rerr = TRUE;			// 受信エラーフラグ
			break;
		}
	}

	//シリアルポート受信バッファクリア 改行コードでbreakしているのでバッファに残っているハズのNullをクリア 
	tcflush(fd1,TCIFLUSH);

	if(werr) ROS_WARN("WRITE ERROR (sendmessage)");
	if(rerr) ROS_WARN("READ ERROR - time out (sendmessage)");

	return (werr<<1) | rerr ;
}

////////////////////////////////////////////////////////////////
//	get_memmap8関数
//	メモリマップのアドレスを指定して、8アドレス分の値を読む関数
//	
//	・引数
//		unsigned char map_add 読み出したいメモリマップのアドレス0～255
//		short value[] 値を返すための配列変数8個分
//
//	・戻り値
//		正常終了は0、異常終了は-1
////////////////////////////////////////////////////////////////

int get_memmap8(unsigned char map_add , short value[]) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], rbuf2[BUFF_SZ];
	char outputc0[5],outputc1[5],outputc2[5],outputc3[5],outputc4[5],outputc5[5],outputc6[5],outputc7[5];
	int pos_sharp = 0, i ,length;

	//送受信メッセージバッファのクリア
	memset(wbuf, 0x00, sizeof(wbuf));
	memset(rbuf, 0x00, sizeof(rbuf));
	memset(rbuf2, 0x00, sizeof(rbuf2));

	//rコマンドのメッセージを作成
	sprintf(wbuf, "r 20%04x 16\r\n", (2048 + (map_add * 2)));

	//送受信！
	sendmessage(wbuf, rbuf);
	if (DEBUG==1) debug_print(wbuf, rbuf);	// デバッグ表示

	//受信結果を整形
	if (rbuf[0] == '\0') rbuf[0] = ' ';	// 1文字目にNullが紛れ込んだときの対策

	for (i = 0; rbuf[i] != '\0'; i++) {
		if (rbuf[i] == '#') pos_sharp = i;
	}

	for (i = 0; rbuf[i + pos_sharp] != '\0'; i++) {
		rbuf2[i] = rbuf[pos_sharp + i];
	}

	length = i;

	// エラーチェック（返答の長さが適切かどうか）
	if(length != 58) return -1;

	// エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
	if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -1;

	// ビッグエンディアン／リトルエンディアン並び替え
	outputc0[0] = rbuf2[11];
	outputc0[1] = rbuf2[12];
	outputc0[2] = rbuf2[8];
	outputc0[3] = rbuf2[9];
	outputc0[4] = 0;

	outputc1[0] = rbuf2[17];
	outputc1[1] = rbuf2[18];
	outputc1[2] = rbuf2[14];
	outputc1[3] = rbuf2[15];
	outputc1[4] = 0;

	outputc2[0] = rbuf2[23];
	outputc2[1] = rbuf2[24];
	outputc2[2] = rbuf2[20];
	outputc2[3] = rbuf2[21];
	outputc2[4] = 0;

	outputc3[0] = rbuf2[29];
	outputc3[1] = rbuf2[30];
	outputc3[2] = rbuf2[26];
	outputc3[3] = rbuf2[27];
	outputc3[4] = 0;

	outputc4[0] = rbuf2[35];
	outputc4[1] = rbuf2[36];
	outputc4[2] = rbuf2[32];
	outputc4[3] = rbuf2[33];
	outputc4[4] = 0;

	outputc5[0] = rbuf2[41];
	outputc5[1] = rbuf2[42];
	outputc5[2] = rbuf2[38];
	outputc5[3] = rbuf2[39];
	outputc5[4] = 0;

	outputc6[0] = rbuf2[47];
	outputc6[1] = rbuf2[48];
	outputc6[2] = rbuf2[44];
	outputc6[3] = rbuf2[45];
	outputc6[4] = 0;

	outputc7[0] = rbuf2[53];
	outputc7[1] = rbuf2[54];
	outputc7[2] = rbuf2[50];
	outputc7[3] = rbuf2[51];
	outputc7[4] = 0;

	//16進数を数値に変換 -> short(2byte)に格納 (ここの処理の時間、一度チェックした方がいいかも)
	value[0] = strtol(outputc0, NULL, 16);
	value[1] = strtol(outputc1, NULL, 16);
	value[2] = strtol(outputc2, NULL, 16);
	value[3] = strtol(outputc3, NULL, 16);
	value[4] = strtol(outputc4, NULL, 16);
	value[5] = strtol(outputc5, NULL, 16);
	value[6] = strtol(outputc6, NULL, 16);
	value[7] = strtol(outputc7, NULL, 16);

	return 0;
}

////////////////////////////////////////////////////////////////
//	get_memmap関数
//	メモリマップのアドレスを指定して、値を読む関数
//	
//	・引数
//		HANDLE hCom	通信に使用するハンドル
//		unsigned char map_add 読み出したいメモリマップのアドレス0～255
//
//	・戻り値
//		メモリマップの値(符号付2バイト) ただし、return -32678はエラー
////////////////////////////////////////////////////////////////

short get_memmap(unsigned char map_add) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], rbuf2[BUFF_SZ], outputc[4];
	short value;
	int pos_sharp = 0, i ,length;

	//送受信メッセージバッファのクリア
	memset(wbuf, '\0', sizeof(wbuf));
	memset(rbuf, '\0', sizeof(rbuf));
	memset(rbuf2, '\0', sizeof(rbuf2));

	//rコマンドのメッセージを作成
	sprintf(wbuf, "r 20%04x 02\r\n", (2048 + (map_add * 2)));

	//送受信！
	sendmessage(wbuf, rbuf);
	if (DEBUG==1) debug_print(wbuf, rbuf);	// デバッグ表示

	//受信結果を整形
	if (rbuf[0] == '\0') rbuf[0] = ' ';	// 1文字目にNullが紛れ込んだときの対策

	for (i = 0; rbuf[i] != '\0'; i++) {
		if (rbuf[i] == '#') pos_sharp = i;
	}

	for (i = 0; rbuf[i + pos_sharp] != '\0'; i++) {
		rbuf2[i] = rbuf[pos_sharp + i];
	}

	length = i;
	// 返答の長さが適切かどうか
	if(length != 16) return -32768;

	// エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
	if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -32768;

	outputc[0] = rbuf2[11];
	outputc[1] = rbuf2[12];
	outputc[2] = rbuf2[8];
	outputc[3] = rbuf2[9];

	//16進数を数値に変換 -> short(2byte)に格納
	value = strtol(outputc, NULL, 16);

	return value;
}

////////////////////////////////////////////////////////////////
//	put_memmap関数
//	メモリマップのアドレスを指定して、値を書き込む関数
//	
//	・引数
//		HANDLE hCom 通信に使用するハンドル
//		unsigned char map_add 書き込みたいメモリマップのアドレス0～255
//		short value 書き込む値(符号付2バイト)
//
//	・戻り値(sendmessageそのまま)
//		0	メッセージの送受信に成功
//		1	メッセージの送信に失敗
//		2	メッセージの受信に失敗
//		3	メッセージの送受信に失敗
////////////////////////////////////////////////////////////////

int put_memmap(unsigned char map_add, short value) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], value_1,value_2;
	int err=0;

	//送受信メッセージバッファのクリア
	memset(wbuf, 0, sizeof(wbuf));
	memset(rbuf, 0, sizeof(rbuf));

	value_1 = value & 0x00FF;
	value_2 = (value >> 8) & 0x00FF;

	//wコマンドのメッセージを作成
	sprintf(wbuf, "w 20%04x %02x %02x\r\n", (2048 + (map_add * 2)), value_1 & 0x000000FF, value_2 & 0x000000FF);

	//送受信！
	err = sendmessage(wbuf, rbuf);
	if (DEBUG==1) debug_print(wbuf, rbuf);	// デバッグ表示

	return err;
}

void read_vsrc(void){

	int error=0;
	short value1[8],value2[8];

	error = get_memmap8(238,value1);
	if (error==0){
		r_mode = value1[0];	//238
		r_voltage = value1[1];	//239
		r_ctltype = value1[2];	//240
	}else{
		ROS_ERROR("get_memmap (238) error");
	}
	
	error = get_memmap8(248,value2);
	if (error==0){
		r_map = value2[0];	//248 map
		r_torque = value2[3];	//251 torque
		r_idle = value2[5];	//253
	}else{
		ROS_ERROR("get_memmap (248) error");
	}

	ROS_INFO("mode:%d | voltage:%d[mV] | map:%d | r_torque:%d <=> w_torque:%d | idle:%d | Ctl_type:%d",r_mode,r_voltage,r_map,r_torque,w_torque,r_idle,r_ctltype);
}

void read_voltage_test(void){
	r_voltage = get_memmap(239);
	ROS_INFO("VOLTAGE=%d",r_voltage);
}

int write_vsrc(void){

	int tmp1,tmp2,err;
	short stk_rx,stk_ry,stk_lx,stk_ly;
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ];

	err = 0;
	err = put_memmap(251, w_torque);
	if (err != 0) ROS_ERROR("w_torque : write error");

	//送受信メッセージバッファのクリア
	memset(wbuf, 0, sizeof(wbuf));
	memset(rbuf, 0, sizeof(rbuf));
	
	// ボタンの書き込み変数作成
	tmp1 = b_L2 + b_R2 * 2 + b_L1 * 4 + b_R1 * 8 + b_triangle * 16 + b_circle * 32+ b_cross * 64 + b_square * 128;
	tmp2 = b_select + b_L3 * 2 + b_R3 * 4 + b_start * 8 + b_up * 16 + b_right * 32+ b_down * 64 + b_left * 128;

	//VS-RC003コマンド生成
	sprintf(wbuf,"w 2009e2 %02x %02x 00 00 00 00 %02x %02x %02x %02x %02x %02x %02x %02x\r\n", tmp1,tmp2,s_rx&0x00ff,(s_rx>>8)&0x00ff,s_ry&0x00ff,(s_ry>>8)&0x00ff,s_lx&0x00ff,(s_lx>>8)&0x00ff,s_ly&0x00ff,(s_ly>>8)&0x00ff);
	//                     ==(241)== -242- -243- ==(244)== ==(245)== ==(246)== ==(247)==

	//送受信！
	if (w_torque==TRUE){				// トルクオンの時だけ送信
		err = 0;
		err = sendmessage(wbuf, rbuf);
		if (DEBUG==1) debug_print(wbuf, rbuf);	// デバッグ表示
		if (err != 0) ROS_ERROR("keypad info : write error");
	}
	return err;
}

void vsrc_ctl_callback(const ros2vsrc::VsrcControl& vsrcctl){

	w_torque 	= vsrcctl.w_torque;

	b_left		= vsrcctl.b_left;
	b_down		= vsrcctl.b_down;
	b_right		= vsrcctl.b_right;
	b_up		= vsrcctl.b_up;
	b_start		= vsrcctl.b_start;
	b_R3		= vsrcctl.b_R3;
	b_L3		= vsrcctl.b_L3;
	b_select	= vsrcctl.b_select;
	b_square	= vsrcctl.b_square;
	b_cross		= vsrcctl.b_cross;
	b_circle	= vsrcctl.b_circle;
	b_triangle	= vsrcctl.b_triangle;
	b_R1		= vsrcctl.b_R1;
	b_L1		= vsrcctl.b_L1;
	b_R2		= vsrcctl.b_R2;
	b_L2		= vsrcctl.b_L2;
	s_lx		= vsrcctl.s_lx;
	s_ly		= vsrcctl.s_ly;
	s_rx		= vsrcctl.s_rx;
	s_ry		= vsrcctl.s_ry;
}

int main(int argc, char **argv)
{
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ];
	sensor_msgs::BatteryState battery_state;
	ros2vsrc::VsrcState vsrc_state;

	ros::init(argc, argv, "vsrc_connect");
	ros::NodeHandle n;

	// シリアルポート名をパラメータからセット
	std::string serialdev;
  	ros::NodeHandle pn("~");
	pn.param<std::string>("serialdev", serialdev, SERIAL_PORT);
	ROS_INFO("Set Serial Port : %s",serialdev.c_str());

	//Publisher
	ros::Publisher battery_pub = n.advertise<sensor_msgs::BatteryState>("battery_state",10);
	ros::Publisher vsrc_pub = n.advertise<ros2vsrc::VsrcState>("vsrc_state",10);

	//Subscriber
	ros::Subscriber vsrc_sub = n.subscribe("vsrc_control", 10, vsrc_ctl_callback);

	// シリアルオープン
	fd1=open_serial(serialdev.c_str());
	if(fd1<0){
		ROS_ERROR("Serial Fail: cound not open %s", serialdev.c_str());
		ros::shutdown();
	}

	ros::Rate loop_rate(100);

	//送受信メッセージバッファのクリア
	memset(wbuf, 0, sizeof(wbuf));
	memset(rbuf, 0, sizeof(rbuf));

	//VS-RC003の制御をコントローラからシリアル通信に切り替える
	ROS_INFO("Start VS-RC003 Connect & Control");

	sprintf(wbuf, "w 2009e0 00 00\r\n");
	//メッセージの送受信。通信が成功しなかったらプログラムを終了 (VS-RC003初回起動時は失敗することがある)
	if (sendmessage(wbuf, rbuf)) {
		ROS_ERROR("Cannot send massage to VS-RC003");
		close(fd1); // ポートクローズ
		return 0;
	}

	while (ros::ok()){	// ここでメインループ！
		read_vsrc();				// VS-RC003からメモリ読み出し

		battery_state.voltage = (float)(r_voltage/1000.0);
		battery_pub.publish(battery_state);	// バッテリー情報を配信

		vsrc_state.r_mode = r_mode;
		vsrc_state.r_map = r_map;
		vsrc_state.r_ctltype = r_ctltype;
		vsrc_state.r_torque = r_torque;
		vsrc_state.r_idle = r_idle;
		vsrc_pub.publish(vsrc_state);		// センサー情報を配信

		write_vsrc();				// VS-RC003のメモリ書き込み

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
