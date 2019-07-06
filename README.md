# ros2vsrc
ROSを用いて、VS-RC003をシリアル経由で制御するパッケージ
vsrc_controlノードとvsrc_connectノードで構成されます。

## vsrc_control
joy_nodeからジョイスティックのトピック(/joy)をsubscribeし、VS-RC003用のコマンドに変換し、vsrc_connectノードに対してPublishするノード

## vsrc_connect
VS-RC003用のコマンドをvsrc_controlからsubscribeし、シリアルポートを通じてVS-RC003と通信するノード

## 使用例

RaspberryPiに接続されたVS-RC003をLANを経由して、遠隔のワークステーションからコントロールします。

### 事前準備
RaspberryPiのGPIOをVS-RC003のCN6もしくはCN7のTxD,RxD,GNDにそれぞれ直結します。

### 実行方法
vsrc_connectノードをRaspberryPi上で実行します。
```
rosrun ros2vsrc vsrc_connect
```

ワークステーション側にXBOX360のコントローラを接続し、joy_nodeノードを起動します。
```
rosrun joy joy_node
```

同じくワークステーション上でvsrc_controlノードを立ち上げます。
```
rosrun ros2vsrc vsrc_control
```

roscoreはどちらに置いても構いません。
これで、LAN(WiFi)経由で、VS-RC003の制御が出来ます。
