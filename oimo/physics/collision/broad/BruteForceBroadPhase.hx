package oimo.physics.collision.broad;

import oimo.physics.collision.broad.Proxy;

import oimo.physics.collision.shape.Shape;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;

/**
	 * 総当りアルゴリズムを使用して広域衝突判定を行うクラスです。
	 * <strong>このアルゴリズムは速度検証以外には非推奨です。</strong>
	 * 総当り判定は形状の数に対し、常に O(n^2) の計算量を要求するため、
	 * 形状の増え方に比べ、負荷の増え方が非常に高くなります。
	 * @author saharan
	 */
class BruteForceBroadPhase extends BroadPhase
{
    private var proxyPool : Array<Proxy>;
    private var numProxies : Int;
    
    /**
		 * 新しく BruteForceBroadPhase オブジェクトを作成します。
		 */
    public function new()
    {
        super();
        proxyPool = new Array<Proxy>();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function addProxy(proxy : Proxy) : Void{
        proxyPool[numProxies] = proxy;
        numProxies++;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function removeProxy(proxy : Proxy) : Void{
        var idx : Int = -1;
        for (i in 0...numProxies){
            if (proxyPool[i] == proxy) {
                idx = i;
                break;
            }
        }
        if (idx == -1) {
            return;
        }
        for (j in idx...numProxies - 1){
            proxyPool[j] = proxyPool[j + 1];
        }
        proxyPool[numProxies] = null;
        numProxies--;
    }
    
    override private function collectPairs() : Void{
        numPairChecks = numProxies * (numProxies - 1) >> 1;
        for (i in 0...numProxies){
            var p1 : Proxy = proxyPool[i];
            var s1 : Shape = p1.parent;
            for (j in i + 1...numProxies){
                var p2 : Proxy = proxyPool[j];
                var s2 : Shape = p2.parent;
                if (
                    p1.maxX < p2.minX || p1.minX > p2.maxX ||
                    p1.maxY < p2.minY || p1.minY > p2.maxY ||
                    p1.maxZ < p2.minZ || p1.minZ > p2.maxZ ||
                    !isAvailablePair(s1, s2)) {
                    {
						continue;
                    }
                }
                addPair(s1, s2);
            }
        }
    }
}

