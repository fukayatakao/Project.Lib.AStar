using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Project.Astar {
    /// <summary>
    /// 探索空間はこのinterfaceを継承する
    /// </summary>
    public interface IMap {
        INode GetNode(int id);
        // ゴールまでの予想コストを計算する
        float calcCostToGoal(INode goal, INode nodeId);
        // 隣接ノードに移動したときのスタートからのコストを計算する
        float calcCostFromStart(INode current, INode neighborNode, float fromCost);
        // 隣接ノードへの移動が閉ざされているか
        bool IsClosure(INode current, INode neighborNode);
    }
    /// <summary>
    /// 探索空間内のノードはこのinterfaceを継承する
    /// </summary>
    public interface INode {
        //ノードID
        int Id{ get; set; }
        //隣接ノードのリスト
        List<INode> Neighbors{ get; set; }
        //スタート位置からのコスト
        float GetCost();
    }

    public enum RouteState : int {
        STATE_NONE,
        STATE_OPEN,
        STATE_CLOSE,
    };

    public class Route {
        //ノードID
        public int Id;
        //親経路
        public Route Parent;
        //到達コスト
        public float CostFromStart;
        //ヒューリスティクスコスト
        public float CostToGoal;
        //到達コスト + ヒューリスティクスコスト
        public float CostTotal;
        //ノードの状態(openリストに居る or closeリストに居る or どちらにもいない)
        public RouteState Status;

        /// <summary>
        /// パラメータを一括でセット
        /// </summary>
        public virtual void Reset(Route parent, float formStart, float toGoal, RouteState state) {
            Parent = parent;
            CostFromStart = formStart;
            CostToGoal = toGoal;
            CostTotal = CostFromStart + CostToGoal;
            Status = state;
        }
    }


    public class AStarSearch<T> where T : Route, new() {

        //経路計算の結果
        public enum Result {
			RESULT_NOT_FOUND,				//経路なし
			RESULT_IN_PROGRESS,				//計算中
			RESULT_ARRIVE					//探索終了
		};
		//探索空間の定数値群(gridの最大数が64x64までと想定)
		const int ROUTE_BANK_MAX = 32 * 32;		//経路計算で使用可能なノード領域(足りないときはメモリ拡張するので問題ない)

		//オープンリスト
		LinkedList<T> open_ = new LinkedList<T>();
		//探索済み空間の情報(クローズリスト)
		Dictionary<int, T> hashTable_ = new Dictionary<int, T>();

//デバッグ用にハッシュテーブルを取得できるようにする
#if UNITY_EDITOR
		public Dictionary<int, T> HashTable{ get { return hashTable_; } }
#endif
		//探索済経路のメモリ領域
		static T[]	routeArray_ = new T[ROUTE_BANK_MAX];
        static int routeArrayCount_;

        /// スタート経路
        T start_;
        /// ゴールノード
        INode goal_;
        //探索空間
        IMap waypointMap_;


		//@note 始点と終点から経路探索。中継点使うのはそのうち。
		/// <summary>
		/// 経路探索
		/// </summary>
		public List<int> SearchPath(IMap map, int start, int goal){
			Init (map);

			Startup (start, goal);

			Result ret = Calculate ();
			//ゴールに到達するか経路なしと判明するまで探索する
			while (ret == Result.RESULT_IN_PROGRESS) {
				ret = Calculate ();
			}


			List<int> path = new List<int> ();
			//経路が存在したら
			if(ret != Result.RESULT_NOT_FOUND)
				Routing (ref path);

			return path;
		}


        //経路探索を分解して実行できるようにpublicで外から呼べるようにしておく
        /// <summary>
        /// 探索マップを設定
        /// </summary>
		public void Init(IMap map){
			waypointMap_ = map;
		}

		/// <summary>
		/// スタートからゴールまでの経路を収集する
		/// </summary>
		public void Routing(ref List<int> path) {
            T route = open_.First.Value;
			while(route.Parent != null) {
				route = (T)route.Parent;
				path.Add(route.Id);
			}
        }


		/// <summary>
		/// 探索開始準備
		/// </summary>
		public void Startup(int startId, int goalId) {
			routeArrayCount_ = 0;
			//経路計算用領域を初期化
			hashTable_.Clear();
			open_.Clear();
			//スタートとゴールを入れ替えて計算する->経路情報を収集するときにスタートからゴールへ向けて順に経路を拾うため
			start_ = allocNewRoute(goalId);
			goal_ = waypointMap_.GetNode(startId);

			//スタートノードをOpenリストへ追加。経路コストも計算。
			start_.Reset(null, 0f, waypointMap_.calcCostToGoal(goal_, waypointMap_.GetNode(goalId)), RouteState.STATE_OPEN);
			open_.AddLast(start_);
		}

        /// <summary>
        /// 移動経路が閉ざされているか
        /// </summary>
        protected virtual bool IsClosure(T current, INode currentNode, INode neighbor) {
            //前の状態に関係なくノード間がつながっていれば移動可能とみなす
            return waypointMap_.IsClosure(currentNode, neighbor);
        }


        /// <summary>
        /// 経路計算(１ステップ分)
        /// </summary>
        public Result Calculate() {
			//Openリストが空になっていたらゴールまでの経路なしと判定
			if(open_.Count == 0) {
				return Result.RESULT_NOT_FOUND;
			}

            //openリスト最初の一番コストが低いノードを取り出す(openリストはコスト順に並んでいるので)
            T route = open_.First.Value;
			//取り出したノードがゴールなら終了
			if(route.Id == goal_.Id) {
				//TRACE("goal\n");
				return Result.RESULT_ARRIVE;
				//ゴールでなければCloseリストへ移動
			} else {
				open_.RemoveFirst ();
				route.Status = RouteState.STATE_CLOSE;
			}


			INode node = waypointMap_.GetNode(route.Id);

			//隣接ノードチェック
			for(int i = 0, max = node.Neighbors.Count; i < max; i++)
			{
				//通行不可なルートの場合は何もしない
				if(IsClosure(route, node, node.Neighbors[i]))
                    continue;

				//コストを計算する
				float fromStart = waypointMap_.calcCostFromStart(node, node.Neighbors[i], route.CostFromStart);
				float toGoal = waypointMap_.calcCostToGoal(goal_, node.Neighbors[i]);
                //openリストとcloseリストの中にnodeMが含まれているか検査

                int nextId = node.Neighbors[i].Id;
                //OpenリストにもCloseリストにもない場合はOpenリストに追加
                if (!hashTable_.ContainsKey(nextId)) {
                    //新規に領域を割り当て
                    T newRoute = allocNewRoute(nextId);
					//コスト順に並ぶようにリストへ挿入
					newRoute.Reset(route, fromStart, toGoal, RouteState.STATE_OPEN);
					insertOpenList(newRoute);
				} else {
                    T actualRoute = hashTable_[nextId];

					//Openリストにある場合は経路コストが前回記録分よりも低い場合は親を書き換える
					if(actualRoute.Status == RouteState.STATE_OPEN) {
						if(fromStart < actualRoute.CostFromStart) {
							deleteOpenList(actualRoute);
							//コスト順に並ぶようにリストへ挿入
							actualRoute.Reset(route, fromStart, toGoal, RouteState.STATE_OPEN);
							insertOpenList(actualRoute);
						}
                    }

					//Closeリストにある場合は経路コストが前回記録分よりも低い場合は親を書き換えてOpenリストに移動する
					if(actualRoute.Status == RouteState.STATE_CLOSE) {
						if(fromStart < actualRoute.CostFromStart) {
							actualRoute.Reset (route, fromStart, toGoal, RouteState.STATE_OPEN);
							//コスト順に並ぶようにリストへ挿入
							insertOpenList(actualRoute);

						}

					}
				}
			}

			return Result.RESULT_IN_PROGRESS;
		}


        /// <summary>
        /// Routeデータの新規作成
        /// </summary>
        T allocNewRoute(int id) {
			if (routeArrayCount_ >= routeArray_.Length) {
				//リサイズが発生したらエラーを一応出しておく。頻繁に出るなら対策必要。
				Debug.Assert (false, "route data resize");
				//2倍で確保しなおし
				System.Array.Resize (ref routeArray_, routeArrayCount_ * 2);
			}
			//@note 毎度newしてもいいけどメモリが断片化するから予め確保した領域から割り当てる
			if (routeArray_ [routeArrayCount_] == null)
				routeArray_ [routeArrayCount_] = new T();
			//ハッシュテーブルに新規領域を割り当てる
			hashTable_[id] = routeArray_[routeArrayCount_];
			routeArrayCount_++;

			//値を設定
			hashTable_[id].Id = id;

			return hashTable_[id];
		}

		/// <summary>
		/// 先頭ノードから大小比較して昇順になるように挿入する
		/// </summary>
		void insertOpenList(T value) {
			//openリストが一時的に空になったときの対策
			if (open_.Count == 0) {
				open_.AddFirst (value);
				return;
			}

			LinkedListNode<T> route = open_.First;
			for(int i = 0, max = open_.Count; i < max; i++) {
				//目標のノードが見つかったら
				if(route.Value.CostTotal >= value.CostTotal){
					break; 
				}
				route = route.Next;
			} 
			if (route == null) {
				open_.AddLast (value);
			} else {
				//挿入して終わり
				open_.AddBefore (route, value);
			}
		}
		/// <summary>
		/// 要素を探して削除
		/// </summary>
		void deleteOpenList(T value){
			LinkedListNode<T> route = open_.First;
			for(int i = 0, max = open_.Count; i < max; i++) {
				//目標のノードが見つかったら
				if(route.Value.Id == value.Id){
					open_.Remove (route);
					break;
				}
			}
        }












        /*/// <summary>
        /// ゴールまでのコストを計算する
        /// </summary>
        float calcCostToGoal(int nodeId) {
            Vector3 n = waypointMap_.GetNode(nodeId).position;
            Vector3 g = goal_.position;
            //とりあえずマンハッタンで
            return Mathf.Abs(n.x - g.x) + Mathf.Abs(n.z - g.z);
        }
        /// <summary>
        /// 開始ノードから隣接ノードまでのコストを計算する
        /// </summary>
        float calcCostFromStart(SplineDebugDraw.Node node, int neighborId, float fromCost) {
            return node.neighbor[neighborId].param.cost + fromCost;
        }*/

    }
}
