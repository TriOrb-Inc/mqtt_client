## 動作環境
自律移動pkgのインストールされたJetson <br>
TriOrb-ROS2-Types と mqtt_client がビルドされていること 

## 実行手順
1. topics.txt にテストしたい型を書く
1. python make_topic.py を実行. 以下が編集される.
    1. params/params.ros2.yaml
    1. html/topic_definition.js
    1. html/subpub.js
1. sh run_tester.sh を実行
1. ip:8080 をブラウザで開いて, testerコンテナ内(srcディレクトリ下)で python node.py を実行(初回は自動で実行される)
1. topics.txt に書かれた型を上から順に publish(jetson) -> subscribe(ブラウザ) -> publish(ブラウザ) -> subscribe(jetson) をn回繰り返す.
    1. クラス変数にはpublish(jetson)ごとにランダムな値が代入される(配列の場合は1~5要素ランダムで挿入)
    1. ブラウザ側はsubscribeしたtopicをそのまま返す
    1. publish(jetson) と subscribe(jetson) の値が一致すればOK
1. check_result.txtに確認結果が出力される

※ publishする型を増やしたり, 変えたい場合はmake_topic.pyの再実行が必要だが, 減らす場合はtopics.txtの変更のみでOK

### Q&A
Q. 実行が止まる <br>
A. 何らかの原因でブラウザ側からの返答がsubscribeできていない可能性が高い <br>
- make_topic.py を実行し忘れている
- mqtt_clientを（再）起動していない
- 通信が切断された
- mqtt_client側の実装にバグがある
- ros2mqtt.jsにバグがある

Q. NGが出ているのはなぜ？
A. pub/subの値が異なる場合, node.py のログに表示するので各変数を比較する. mqtt_client側で実装されていない可能性が高い.