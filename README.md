# mavlink-com
単一ドローン操作のための簡易的な地上局ソフトの開発

## 機能
- ドローンからのデータ取得
    - 姿勢
    - 高度
    - 緯度経度
    - 対地速度
    - バッテリー残量
- 機体モードの変更
- アーミング
- 高度指定の離陸
- 着陸
- ピッチ変更
- 移動（機体座標、絶対座標）
- Waypointによる移動（絶対座標）

![image](https://github.com/yunTum/mavlink-com/assets/34528586/d840d468-1158-45c4-91c0-0de38ce61da6)

## 実行
### GUI表示
接続ポート指定
```
python main.py gui --port 5762
```
COM指定
```
python main.py gui --comport com4
```

### CUIでのテストコマンド
第二引数をtestに変更
```
python main.py test --port 5762
```
