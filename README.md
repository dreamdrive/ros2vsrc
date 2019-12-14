# ros2vsrcについて
VS-RC003(Vstone社製ロボット用制御ボード)をシリアルポート経由で、ROSを用いて制御するためのパッケージです。

![ros2vsrc](http://dream-drive.net/images/ros2vsrc.jpg "ros2vsrc")


20190728更新 : vsrc_connect2とvsrc_control2を追加  
20191214更新 : vsrc_connect2とvsrc_control2を、vsrc_connectとvsrc_controlに変更。従来のvsrc_connectとvsrc_controlはリタイア(.old)。

MIT License / Copyright (c) 2019 Hirokazu Onomichi

## vsrc_connect (ノード)
VS-RC003を制御するためのノードです。

* カスタムメッセージVsrcControlをSubscribします。  
VsrcControlはコントローラーのボタンの状態とトルクON/OFFの情報で構成され、Subscribした情報をもとにVS-RC003のメモリを書き換えます。
* VS-RC003の状態としてボードのメモリ上の情報を定期的にカスタムメッセージVsrcStateとBatteryStateへPublishします。  
接続先のノードから電源電圧やモーションの再生状況を監視するすることが出来ます。  
メッセージの詳しい内容は後記もしくは、msgファイルを参照してください。

## vsrc_control (サンプルノード)
公式joyパッケージのjoy_nodeからジョイスティックのトピック(/joy)をsubscribeし、VsrcControlのメッセージをpublishするサンプルノードです。
Xbox360のコントローラを例に作成しています。

## VsrcState.msgとVsrcControl.msg (メッセージ)
VsrcStateとVsrcControl、2つのカスタムメッセージを用意しました。
VsrcStateはVS-RC003の状態を表すメッセージ、VsrcControlはVS-RC003に制御のパラメータ(コントローラ情報)を送るメッセージになっています。
具体的な内容は下記画像の通りです。

![ROS_TOPIC](http://dream-drive.net/images/ros_topic_vsrc.png "メッセージの中身")

## 旧vsrc_connectノードと旧vsrc_controlノードについて
互換性がありません。

# 使用例1 - PCにVS-RC003もジョイスティックも直接接続する場合

PCに接続した、XBOX360コントローラーをVS-RC003のコントローラーとして使用するサンプルlaunchです。シリアルポートのデバイス名をパラメータ"serialdev"にセットして起動してください。

```
$ roslaunch ros2vsrc joystick2vsrc.launch serialdev:=/dev/ttyUSB0
```


# 使用例2 - VS-RC003を接続したRaspberryPiを、ネットワーク越しにPCに接続したジョイスティックから操作する

## 事前準備
RaspberryPiのGPIOを、VS-RC003のCN6もしくはCN7のTxD,RxD,GNDに、それぞれ直結します。

```
export ROS_MASTER_URI=http://(RaspberryPiのIP):11311/
export ROS_IP = (自分のIP)
```
を、ノード起動前に、両側で正しく実行すること。

※ ROS MASTERをRaspberryPi側に設定する例です。  
※ 自分のIPとは、RaspPiはRasPiのIP、PCはPCのIPです。

## 実行方法
vsrc_connectノードをRaspberryPi上で実行します。
```
$ roscore &
$ rosrun ros2vsrc vsrc_connect _serialdev:=/dev/ttyAMA0
```
※ 「/dev/ttyAMA0」は、RaspberryPiのGPIOのシリアルポート名です。

PC側にXBOX360のコントローラを接続し、joy_nodeノードとvsrc_controlノードを起動します。
```
$ rosrun joy joy_node &
$ rosrun ros2vsrc vsrc_control
```
起動すると、rqt_graphの様子はこんな感じになります。
![ROS_NODE](http://dream-drive.net/images/ros_node_vsrc.png "ノードの状態")

これで、LAN/WiFi経由でROSを使ってVS-RC003の制御出来ます。  
また、ROS側で動作計画を立ててVS-RC003にモーションの指令を出す、といったVS-RC003だけでは実行できないより高度な自律動作をさせることも出来るかもしれません。

# 謝辞
vsrc_connectの作成に至っては、こちらの記事を参考にさせていただきました。ありがとうございます。  
> https://qiita.com/srs/items/efaa8dc0a6d580c7c423
