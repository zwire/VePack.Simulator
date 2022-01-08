# VePack.Simulator

![sim.png](/sim.png)

### Requirements

* VeBotsメンバーであること
* .NET5/.NET6 開発環境 (Visual Studio等)

### Installation

* AirSim ...  
1. Python パッケージ
```
pip install msgpack-rpc-python
pip install airsim
```
2. [AirSim ダウンロードページ](https://github.com/Microsoft/AirSim/releases) のAssetsから好きなzipファイルを落として展開.
3. exeを開いてみる. おそらくDirectXについてのエラーが出るので, [DirectX エンドユーザーランタイムを手動でインストールする方法](https://faq.tsukumo.co.jp/index.php?solution_id=1321)で解決.
4. [settings.json](/AirSim/settings.json) を展開したフォルダのexeがあるところに配置.

* VePack ... 最新版は私のプライベートリポジトリから。aresにも一応置いてるのでcollabolatorでない場合はそちらから。

### About VePack
* Utilities ... IO とか Geometry とか
* Connectors ... sender / receiver のベースクラスとセンサ系
* Plugin/Navigation ... 位置方位をもとにマップ系の情報を返してくれる
* Plugin/Controllers ... 制御アルゴリズムの実装
* Plugin/Filters ... センサの後処理向けのフィルタ類
