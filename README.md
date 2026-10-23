# glim: masterからの変更と動作説明

この文書は、GLIM本体の大域補正モード、手動補正、IMU速度の座標変換、コールバックの寿命管理について、標準の`master`と現在の実装の違いを説明します。一般的な導入説明は[既存のREADME](README.md)を参照してください。

## 1. 比較した版と対象範囲

2026-10-22時点のローカルGitとソースを確認しました。

| 項目 | 比較対象 |
| --- | --- |
| リポジトリ | `glim`。`origin`は`kawano4114/glim`のfork |
| 基準 | ローカル`master` / `origin/master`: `2262aafa2369acff6afab2268a7743aec7a7fa49` |
| 現在のブランチ / HEAD | `trigger` / `09ba82359bea3882690a0339e534c838dfd316ea` |
| 文書化する現在の状態 | HEADに、作業ツリー上の未コミット修正を加えた状態 |

`master`は現在のHEADの祖先で、コミット済みの追加変更は次の2件です。

| コミット | 内容 |
| --- | --- |
| `f5559fa` | 3モード、手動全グラフ補正、設定・APIを追加 |
| `09ba823` | グローバル地図の配信間隔設定・説明を追加 |

IMU座標変換の修正、PoseGraphのモード検査、コールバック解除待ちなどは、確認時点では未コミットです。HEADだけをcheckoutすると、この文書が説明する修正の一部は含まれません。

比較には`git diff master`を使い、コミット済み変更と未コミット変更を合わせて確認しました。リモートのfetchやWeb照合はしていないため、この`master`を上流プロジェクトの最新状態とは断定しません。

### 2つのリポジトリの分担

| リポジトリ | 担当する処理 |
| --- | --- |
| この`glim` | サブマップ・グラフの蓄積、最適化方式の選択、全グラフLM、最適化結果の保持 |
| `frontier_glim`（ROSパッケージ名は`glim_ros`） | Empty購読、受信INFO、ROSの終了処理、`~/map`の定期配信・キャッシュ |

`glim`単独ではROSのEmpty購読も地図配信も実装しません。両者の関係は[frontier_glim側の変更説明](../frontier_glim/README_CUSTOM_CHANGES.md)も参照してください。

## 2. どこを変更したか

リンク先の実装ファイルには、対応するクラスのヘッダも変更されています。

| ファイル・クラス | masterからの変更 | 動作が変わる場面 |
| --- | --- | --- |
| [global_mapping.cpp](src/glim/mapping/global_mapping.cpp)、[global_mapping.hpp](include/glim/mapping/global_mapping.hpp) | モード読込・検査、Dummy保持、`store_pending_factors()`、`optimize_manual()`、IMU速度変換を追加・修正 | サブマップ追加、保存、手動補正、グラフ読込・復旧 |
| [async_global_mapping.cpp](src/glim/mapping/async_global_mapping.cpp)、[ヘッダ](include/glim/mapping/async_global_mapping.hpp) | モード別要求受付、atomic要求の集約、入力バッチ後の手動処理、完了・失敗ログ、コールバック寿命管理 | 入力処理と補正要求が別スレッドから届くとき |
| [global_mapping_base.hpp](include/glim/mapping/global_mapping_base.hpp) | `optimization_mode()`と`optimize_manual()`を仮想APIとして追加 | Async側からバックエンドの補正方式を判定するとき |
| [global_mapping_pose_graph.cpp](src/glim/mapping/global_mapping_pose_graph.cpp) | モード1以外を起動時に拒否 | PoseGraph用ライブラリを選択するとき |
| [callback_slot.hpp](include/glim/util/callback_slot.hpp) | 登録一覧の排他、実行中コールバックの解除待ち、`CallbackConnections`を追加 | 登録・実行・破棄が並行するとき |
| [standard_viewer.cpp](src/glim/viewer/standard_viewer.cpp)、[standard_viewer_callbacks.cpp](src/glim/viewer/standard_viewer_callbacks.cpp)、[ヘッダ](include/glim/viewer/standard_viewer.hpp) | 所有者に紐づく登録へ変更し、デストラクタで解除 | Viewer終了時 |
| [interactive_viewer.cpp](src/glim/viewer/interactive_viewer.cpp)、[ヘッダ](include/glim/viewer/interactive_viewer.hpp) | 同じくコールバックを解除してからスレッドを終了 | InteractiveViewer終了時 |
| [odometry_estimation_ct.cpp](src/glim/odometry/odometry_estimation_ct.cpp)、[CTヘッダ](include/glim/odometry/odometry_estimation_ct.hpp)、[odometry_estimation_imu.cpp](src/glim/odometry/odometry_estimation_imu.cpp)、[IMUヘッダ](include/glim/odometry/odometry_estimation_imu.hpp) | 共分散計算要求の登録を寿命管理対象に変更 | Odometryオブジェクト破棄後の要求を防ぐとき |
| [CPU設定](config/config_global_mapping_cpu.json)、[GPU設定](config/config_global_mapping_gpu.json) | モード3・反復上限20を追加 | これらの設定を使った起動時 |
| [config_ros.json](config/config_ros.json) | Emptyトピック・地図配信間隔を追加 | 対応するROS側実装がこの設定を読むとき |
| [parameters.md](docs/parameters.md)、[extend.md](docs/extend.md) | パラメータとコールバック登録例を更新 | 設定・拡張モジュールを作成するとき |

## 3. 大域補正の3モード

対象はCPU/GPU設定が使う`libglobal_mapping.so`です。masterでは主に`enable_optimization`でiSAM2とDummyを切り替えていました。現在は、Global Mappingを動かしたまま補正のタイミングを選べます。

| 設定・操作 | モード1: 自動 | モード2: 補正なし | モード3: Emptyによる手動 |
| --- | --- | --- | --- |
| 点群・サブマップ・因子の蓄積 | 行う | 行う | 行う |
| 通常追加時のバックエンド | `ISAM2Ext` | `ISAM2ExtDummy` | `ISAM2ExtDummy` |
| 追加時の大域補正 | iSAM2の更新を行う | 行わない | 行わない |
| 入力が空のときの定期最適化 | 行う。Asyncの既定間隔は5秒 | 行わない | 行わない |
| ViewerのOptimize | iSAM2更新を要求 | 無視 | 無視 |
| ROSのEmpty | ROS側で無視 | ROS側で無視 | 全グラフLMを要求 |
| 保存時の`optimize()` | iSAM2更新 | 未反映因子・Valuesの保持のみ | 未反映因子・Valuesの保持のみ |
| 手動補正後 | 対象外 | 対象外 | 補正結果をDummyへ保持し、補正せず蓄積を再開 |

Dummyは因子と初期値を保持するためのバックエンドです。単にGlobal Mappingを停止するモードではありません。内部のサブマップ生成、相対拘束の計算、点群の保持は継続します。

隣接サブマップの相対拘束を作る登録処理や、Local Mapping側の最適化は、大域姿勢の補正とは別処理です。モード2でもこれらの計算まで一律に停止するわけではありません。

### 設定ファイルと優先順位

使用するファイルは[config/config.json](config/config.json)の`global.config_global_mapping`で選びます。現在の既定は`config_global_mapping_gpu.json`です。CPU版を使うときは、選択先もCPU用ファイルに合わせます。

選択したグローバル設定の`global_mapping`内を変更します。次は関連項目だけの抜粋です。

```json
{
  "global_mapping": {
    "so_name": "libglobal_mapping.so",
    "enable_optimization": true,
    "optimization_mode": 3,
    "manual_optimization_max_iterations": 20
  }
}
```

| 条件 | 実際の扱い |
| --- | --- |
| `optimization_mode`を省略 | フォールバックは1。ただし配布CPU/GPU設定には3を明記 |
| `enable_optimization=false` | 有効なモード指定に対して、実効モードを2へ変更 |
| モードが1・2・3以外 | 初期化時に例外。旧スイッチによるモード2への変更より先に検査 |
| `manual_optimization_max_iterations`省略 | 20 |
| 反復上限が1未満 | モード1・2であっても初期化時に例外 |
| ROS側`enable_global_mapping=false` | Global Mapping自体が作成されず、この3モードは働かない |
| `libglobal_mapping_pose_graph.so`でモード2・3 | 初期化時に例外。3モードを使う場合は`libglobal_mapping.so`を選ぶ |

PoseGraphはモード未指定を含む1だけを受け付けます。従来の`enable_optimization=false`は同実装に残っています。

モードはオブジェクト生成時に読み、Async側も固定値として保持します。実行中にJSONを書き換えても切り替わりません。

ROS launchから起動すると、`frontier_glim/config/config_gazebo`など別の設定ディレクトリが使われます。その場合、このリポジトリの`config/`だけを編集しても起動内容は変わりません。

## 4. Emptyから補正結果反映まで

```text
frontier_glim: GlimROS::optimize_callback()
  └─ モード3なら AsyncGlobalMapping::request_optimization()
       └─ atomic要求フラグをtrue
            └─ Global Mappingワーカーの入力バッチ終了後
                 └─ GlobalMapping::optimize_manual()
                      ├─ 未反映因子・Valuesを保持
                      ├─ 蓄積済みの全グラフ・全ValuesでバッチLM
                      ├─ 誤差の有限性と全サブマップ姿勢を検査
                      ├─ 因子と最適化済み全Valuesを新しいDummyへ保持
                      ├─ 全サブマップのT_world_originを更新
                      └─ 更新コールバック → 完了INFO
```

### いつ始まり、何を最適化するか

- 要求処理は入力キューが空かどうかの条件の外にあります。新しいサブマップの到着や、入力全体の停止を待ちません。
- 境界は、ワーカーが`get_all_and_clear()`で取り出した画像・IMU・サブマップのバッチ全体の終了後です。1サブマップ処理ごとの割り込みではありません。
- 実行時点でバックエンドへ追加済みの全サブマップが対象です。入力キューで待機しているだけのサブマップはまだ対象ではありません。
- 1要求でLMを実行し、ソルバーの終了条件または`manual_optimization_max_iterations`まで進めます。Emptyを20回送って20反復させる方式ではありません。
- 最適化・通常追加・保存は`global_mapping_mutex`で直列化されます。LM実行中にも入力キューへの投入や次回要求の登録は可能です。
- 手動補正は新しい重なり探索を自動実行しません。それまでに作られた因子・拘束を使います。重なり探索の要求自体は、従来どおり入力が空のときに処理されます。

### 複数要求、初回、補正後の追加

| タイミング | 動作 |
| --- | --- |
| 実行開始前に複数要求 | boolで1件へ集約 |
| `exchange(false)`後、補正中に要求 | フラグが再びtrueになり、次回分として残る |
| 次回分が残った状態で入力が到着 | 次の入力バッチを処理してから再補正 |
| 初回要求 | 初回までに蓄積した全グラフを補正 |
| 補正後に新サブマップ追加 | 直前サブマップの補正済み姿勢を基準に新しい姿勢を生成 |
| 2回目以降の要求 | 前回結果と追加済み状態を初期値にして全グラフを再補正 |

通常の要求受付が有効なのはGlobal Mappingワーカー稼働中です。`join()`でワーカーが終了した後の要求を再実行する機構はありません。

### 保存地図の読み込み・重なり探索

空の地図への`load()`は、読み込んだ因子とValuesをそのモードのバックエンドへ取り込みます。既存地図へ別セッションを追加する場合は、因子とValuesを保留し、その後の追加・保存・手動要求などで取り込みます。モード2・3ではその取り込み自体で大域ソルバーを動かしません。ただし拡張コールバックが読み込みセッションの初期姿勢を揃える操作は可能です。

重なり探索はモード2・3でも新しい因子を作って保持し、更新通知を出します。ここでもDummyを使うため、保持済み全体姿勢をソルバーで補正しません。ViewerのRecover要求はモード1だけが受理します。

### ログと失敗時の境界

受信INFOはROS側で出します。成功時の`manual global optimization completed`は、このリポジトリのAsync側が、全サブマップ姿勢の更新と更新コールバックの呼び出しが戻った後に出します。これはROS点群の配信完了を意味しません。

空地図・空グラフなどで`false`を返した場合や、`std::exception`を捕捉した場合はWARNとなり、成功INFOを出しません。処理した要求は消費されるため、失敗した要求の自動再試行はありません。

LMの結果検査と新しいDummyへの格納が成功するまでは、既存`isam2`を交換しません。ただし、事前の`store_pending_factors()`は生きたグラフへ未反映データを追加し、因子グラフのコピーも因子実体の深い複製ではありません。また、交換後の更新コールバックで例外が起きた場合のロールバックはありません。あらゆる失敗で全状態が完全に不変になる保証ではありません。

## 5. IMU変数・拘束を保持する変更

手動補正は`X(i)`のサブマップ姿勢だけを抽出しません。`E`のIMU端点姿勢、`V`の速度、`B`のbiasなど、蓄積した全Valuesと因子をLMへ渡し、結果も全ValuesとしてDummyへ保存します。これにより、次の追加と2回目以降の補正でIMU変数を参照できます。

補正後の追加では、サブマップ原点姿勢はグローバル補正済みでも、`submap->frames`に保存された速度は生成元の座標系にあります。masterの変換はここで座標系を混ぜるため、現在は次の順で処理します。

```text
ローカル速度 = 元の中心フレーム回転の逆 × 元のworld速度
新しいworld速度 = 補正後のサブマップ回転 × ローカル速度
```

元の回転は`origin_frame()->T_world_sensor()`から取得します。通常追加だけでなく、保存地図の欠落変数・拘束を復旧する`recover_graph()`でも同じ変換を使います。

## 6. コールバックを所有者の寿命に合わせる変更

masterの`CallbackSlot`は関数を配列へ保持し、`remove()`は該当項目を空にするだけでした。実行中コールバックが破棄済みのViewerなどへアクセスする問題に対し、現在は次の仕組みにしています。

1. 登録一覧はmutexで保護し、呼び出し時はスナップショットを取得する。
2. 登録ごとに有効フラグと実行ロックを持ち、無効化した登録を再実行しない。
3. `remove()`は無効化後に実行ロックを取得し、別スレッドで進行中の呼び出しが戻るのを待つ。
4. `CallbackConnections`が登録IDを所有し、`clear()`で登録の逆順に解除する。
5. Viewer・Async Global Mapping・Odometryは、状態を破棄する前に`clear()`する。

`clear()`と登録は所有者のライフサイクルを管理するスレッドで行い、解除が終わるまで所有者と登録先slotを生存させます。`clear()`はキュー内の入力を処理し尽くすAPIではありません。ROS入力停止と処理パイプラインのjoinは、ROS側リポジトリが担当します。

この変更が同期する対象はコールバックの登録解除と実行です。コールバック内で使う任意の共有データまで、自動的に排他される仕組みではありません。

## 7. ROSから使う例

関連する`config_ros.json`の設定例です。

```json
{
  "glim_ros": {
    "enable_local_mapping": true,
    "enable_global_mapping": true,
    "optimize_topic": "/glim_ros/optimize",
    "ros_pub_interval_time": 20.0
  }
}
```

Local Mappingを有効にすると内部サブマップが作られ、Global Mappingへ渡されます。モード3で補正を要求する操作は次のとおりです。

```bash
ros2 topic pub --once /glim_ros/optimize std_msgs/msg/Empty '{}'
```

モード1・2はこのEmptyを無視します。モード3の補正は大域地図の状態を更新する処理であり、直後に`~/map`を臨時配信する操作ではありません。

`ros_pub_interval_time`は秒単位の間隔で、20.0は20秒です。ここでは設定値を追加しただけで、周期計測・合成・キャッシュ・publishの実装は`frontier_glim`の`RvizViewer`にあります。大域補正モードと配信周期は独立しています。

## 8. 検証範囲と残る確認項目

この文書は現在のソース、Git差分、呼び出し経路に基づく説明です。文書作成ではビルド、ROS起動、bag再生、最適化実行を行っていません。

- 3モードを同じデータで比較し、モード2・3で保存や入力停止から意図しない補正が起きないこと。
- 回転補正後にサブマップを追加し、IMU速度・拘束が整合したまま再補正できること。
- 要求の集約、補正中の次回要求、空地図・LM失敗後の継続を確認すること。
- Viewer・コンポーネントを処理中に終了して、解放後アクセスやデッドロックがないこと。
- ROS側には、旧スナップショットの地図合成中に補正が完了すると旧地図を配信し、補正済み配信がさらに次周期になる境界条件が残ること。

とくに、完了INFOを受け取ったことと、補正済みPointCloud2を受け取ったことは別々に確認してください。
