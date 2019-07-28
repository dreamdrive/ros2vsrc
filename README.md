# ros2vsrcについて
VS-RC003(Vstone社製ロボット用制御ボード)をシリアルポート経由で、ROSを用いて制御するためのパッケージです。

![ros2vsrc](http://dream-drive.net/images/ros2vsrc.jpg "ros2vsrc")

RaspberryPiにROS kineticをインストールした環境で開発しています。
ROSワークスペースのsrcフォルダ内に置けば、catkin_makeでコンパイル出来ます。
おそらく、Ubuntuが動けばデスクトップPCでも、USBシリアルケーブルで同様に動作させることが可能と思います。

20190728更新 : vsrc_connect2とvsrc_control2を追加

MIT License / Copyright (c) 2019 Hirokazu Onomichi

## vsrc_connect2 (ノード)
VS-RC003を制御するためのノードです。本ノードで、VS-RC003とシリアル通信を行います。

* カスタムメッセージVsrcControlをSubscribします。VsrcControlはコントローラーのボタンの状態とトルクON/OFFの情報で構成され、Subscribした情報をもとにVS-RC003のメモリを書き換えます。
* VS-RC003の状態としてボードのメモリ上の情報を定期的にカスタムメッセージVsrcStateとBatteryStateへPublishします。接続先のノードから電源電圧やモーションの再生状況を監視するすることが出来ます。メッセージの詳しい内容は後記もしくは、msgファイルを参照してください。
(従来、旧vsrc_connectでは、VSRCコマンドそのものを文字列として飛ばしていましたが、必要なものを小分けにしてメッセ―ジ化しました。)

また、テスト動作としては、rqt_topic_monitorおよびrqt_ez_publisherだけでも、全機能を試すことが出来ます。

## vsrc_control2 (ノード)
joy_nodeからジョイスティックのトピック(/joy)をsubscribeし、VsrcControlのメッセージをpublishするサンプルノードです。
Xbox360のコントローラを例に作成しています。
(従来、旧vsrc_controlでは、このノードの中でVSRCコマンドを生成していましたが、今回はVsrcControlメッセージを利用するため、互換性はありません。)

## VsrcState.msgとVsrcControl.msg (メッセージ)
VsrcStateとVsrcControl、2つのカスタムメッセージを用意しました。
VsrcStateはVS-RC003の状態を表すメッセージ、VsrcControlはVS-RC003に制御のパラメータ(コントローラ情報)を送るメッセージになっています。
具体的な内容は下記画像の通りです。

![ROS_TOPIC](http://dream-drive.net/images/ros_topic_vsrc.png "メッセージの中身")

## 旧vsrc_connectノードと旧vsrc_controlノードについて
旧vsrc_controlノードとvsrc_connectも残していますが、互換性はありません。
VSRCの専用コマンドを文字列として送っていたため、専用コマンドを覚えないと使えないという欠点があったため、使用は推奨しません。

# 使用例
RaspberryPiに接続されたVS-RC003をLANを経由して、遠隔のワークステーションからコントロールします。
(RaspberryPiにXBOXコントローラを繋いで、localで実行しても構いません。)

## 事前準備
RaspberryPiのGPIOを、VS-RC003のCN6もしくはCN7のTxD,RxD,GNDに、それぞれ直結します。
PCに繋ぐ場合は、USBシリアルケーブルをUARTのレベルに変換して接続してください。

ROSMASTERは、どちら側に設定しても構いませんが、
```
export ROS_MASTER_URI=http://(roscoreが動いているIP):11311/
export ROS_IP = (自分のIP)
```
を、ノード起動前に、両側で正しく実行すること。
(ローカルでjoy_nodeも立ち上げる場合は設定不要です。)

## 実行方法
vsrc_connect2ノードをRaspberryPi上で実行します。
```
rosrun ros2vsrc vsrc_connect2
```

ワークステーション側にXBOX360のコントローラを接続し、joy_nodeノードを起動します。
```
rosrun joy joy_node
```

同じくワークステーション上でvsrc_controlノードを立ち上げます。
```
rosrun ros2vsrc vsrc_control2
```
起動すると、rqt_graphの様子はこんな感じになります。
![ROS_NODE](http://dream-drive.net/images/ros_node_vsrc.png "ノードの状態")

これで、LAN/WiFi経由でROSを使ってVS-RC003の制御出来ます。
また、ROS側で動作計画を立ててVS-RC003にモーションの指令を出す、といったVS-RC003だけでは実行できないより高度な自律動作をさせることも出来るかもしれません。

# 謝辞
vsrc_connectの作成に至っては、こちらの記事を参考にさせていただきました。ありがとうございます。  
> https://qiita.com/srs/items/efaa8dc0a6d580c7c423
