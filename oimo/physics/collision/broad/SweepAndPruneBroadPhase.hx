package oimo.physics.collision.broad;


import oimo.physics.collision.shape.Shape;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;

/**
	 * Sweep And Prune アルゴリズムを使用して広域衝突判定を行うクラスです。
	 * プロキシの要素は各軸についてソートされ、
	 * 衝突の可能性がある形状のペアを効率的に計算することができます。
	 * ワールドに対し多数の形状がワープおよび高速で移動するような場面では、
	 * このアルゴリズムは好ましくない結果を生み出す可能性があります。
	 * @author saharan
	 */
class SweepAndPruneBroadPhase extends BroadPhase
{
    private var proxyPoolAxis : Array<Array<Proxy>>;
    private var sortAxis : Int;
    private var numProxies : Int;
    
    /**
		 * 新しく SweepAndPruneBroadPhase オブジェクトを作成します。
		 */
    public function new()
    {
        super();
        sortAxis = 0;
        proxyPoolAxis = new Array<Array<Proxy>>();
        proxyPoolAxis[0] = new Array<Proxy>();
        proxyPoolAxis[1] = new Array<Proxy>();
        proxyPoolAxis[2] = new Array<Proxy>();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function addProxy(proxy : Proxy) : Void{
        proxyPoolAxis[0][numProxies] = proxy;
        proxyPoolAxis[1][numProxies] = proxy;
        proxyPoolAxis[2][numProxies] = proxy;
        numProxies++;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function removeProxy(proxy : Proxy) : Void{
        removeProxyAxis(proxy, proxyPoolAxis[0]);
        removeProxyAxis(proxy, proxyPoolAxis[1]);
        removeProxyAxis(proxy, proxyPoolAxis[2]);
        numProxies--;
    }
    
    override private function collectPairs() : Void{
        numPairChecks = 0;
        var proxyPool : Array<Proxy> = proxyPoolAxis[sortAxis];
        var result : Int;
        if (sortAxis == 0) {
            insertionSortX(proxyPool);
            sweepX(proxyPool);
        }
        else if (sortAxis == 1) {
            insertionSortY(proxyPool);
            sweepY(proxyPool);
        }
        else {
            insertionSortZ(proxyPool);
            sweepZ(proxyPool);
        }
    }
    
    private function sweepX(proxyPool : Array<Proxy>) : Void{
        var center : Float;
        var sumX : Float = 0;
        var sumX2 : Float = 0;
        var sumY : Float = 0;
        var sumY2 : Float = 0;
        var sumZ : Float = 0;
        var sumZ2 : Float = 0;
        var invNum : Float = 1 / numProxies;
        var bodyStatic : Int = RigidBody.BODY_STATIC;
        for (i in 0...numProxies){
            var p1 : Proxy = proxyPool[i];
            center = p1.minX + p1.maxX;
            sumX += center;
            sumX2 += center * center;
            center = p1.minY + p1.maxY;
            sumY += center;
            sumY2 += center * center;
            center = p1.minZ + p1.maxZ;
            sumZ += center;
            sumZ2 += center * center;
            var s1 : Shape = p1.parent;
            for (j in i + 1...numProxies){
                var p2 : Proxy = proxyPool[j];
                numPairChecks++;
                if (p1.maxX < p2.minX) {
                    break;
                }
                var s2 : Shape = p2.parent;
                if (
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
        sumX = sumX2 - sumX * sumX * invNum;
        sumY = sumY2 - sumY * sumY * invNum;
        sumZ = sumZ2 - sumZ * sumZ * invNum;
        if (sumX > sumY) {
            if (sumX > sumZ) {
                sortAxis = 0;
            }
            else {
                sortAxis = 2;
            }
        }
        else if (sumY > sumZ) {
            sortAxis = 1;
        }
        else {
            sortAxis = 2;
        }
    }
    
    private function sweepY(proxyPool : Array<Proxy>) : Void{
        var center : Float;
        var sumX : Float = 0;
        var sumX2 : Float = 0;
        var sumY : Float = 0;
        var sumY2 : Float = 0;
        var sumZ : Float = 0;
        var sumZ2 : Float = 0;
        var invNum : Float = 1 / numProxies;
        var bodyStatic : Int = RigidBody.BODY_STATIC;
        for (i in 0...numProxies){
            var p1 : Proxy = proxyPool[i];
            center = p1.minX + p1.maxX;
            sumX += center;
            sumX2 += center * center;
            center = p1.minY + p1.maxY;
            sumY += center;
            sumY2 += center * center;
            center = p1.minZ + p1.maxZ;
            sumZ += center;
            sumZ2 += center * center;
            var s1 : Shape = p1.parent;
            for (j in i + 1...numProxies){
                var p2 : Proxy = proxyPool[j];
                numPairChecks++;
                if (p1.maxY < p2.minY) {
                    break;
                }
                var s2 : Shape = p2.parent;
                if (
                    p1.maxX < p2.minX || p1.minX > p2.maxX ||
                    p1.maxZ < p2.minZ || p1.minZ > p2.maxZ ||
                    !isAvailablePair(s1, s2)) {
                    {
						continue;
                    }
                }
                addPair(s1, s2);
            }
        }
        sumX = sumX2 - sumX * sumX * invNum;
        sumY = sumY2 - sumY * sumY * invNum;
        sumZ = sumZ2 - sumZ * sumZ * invNum;
        if (sumX > sumY) {
            if (sumX > sumZ) {
                sortAxis = 0;
            }
            else {
                sortAxis = 2;
            }
        }
        else if (sumY > sumZ) {
            sortAxis = 1;
        }
        else {
            sortAxis = 2;
        }
    }
    
    private function sweepZ(proxyPool : Array<Proxy>) : Void{
        var center : Float;
        var sumX : Float = 0;
        var sumX2 : Float = 0;
        var sumY : Float = 0;
        var sumY2 : Float = 0;
        var sumZ : Float = 0;
        var sumZ2 : Float = 0;
        var invNum : Float = 1 / numProxies;
        var bodyStatic : Int = RigidBody.BODY_STATIC;
        for (i in 0...numProxies){
            var p1 : Proxy = proxyPool[i];
            center = p1.minX + p1.maxX;
            sumX += center;
            sumX2 += center * center;
            center = p1.minY + p1.maxY;
            sumY += center;
            sumY2 += center * center;
            center = p1.minZ + p1.maxZ;
            sumZ += center;
            sumZ2 += center * center;
            var s1 : Shape = p1.parent;
            for (j in i + 1...numProxies){
                var p2 : Proxy = proxyPool[j];
                numPairChecks++;
                if (p1.maxZ < p2.minZ) {
                    break;
                }
                var s2 : Shape = p2.parent;
                if (
                    p1.maxX < p2.minX || p1.minX > p2.maxX ||
                    p1.maxY < p2.minY || p1.minY > p2.maxY ||
                    !isAvailablePair(s1, s2)) {
                    {
						continue;
                    }
                }
                addPair(s1, s2);
            }
        }
        sumX = sumX2 - sumX * sumX * invNum;
        sumY = sumY2 - sumY * sumY * invNum;
        sumZ = sumZ2 - sumZ * sumZ * invNum;
        if (sumX > sumY) {
            if (sumX > sumZ) {
                sortAxis = 0;
            }
            else {
                sortAxis = 2;
            }
        }
        else if (sumY > sumZ) {
            sortAxis = 1;
        }
        else {
            sortAxis = 2;
        }
    }
    
    private function removeProxyAxis(proxy : Proxy, proxyPool : Array<Proxy>) : Void{
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
    }
    
    private function insertionSortX(proxyPool : Array<Proxy>) : Void{
        if (numProxies == 1) 
            return;
        for (i in 1...numProxies){
            var insert : Proxy = proxyPool[i];
            if (proxyPool[i - 1].minX > insert.minX) {
                var j : Int = i;
                do{
                    proxyPool[j] = proxyPool[j - 1];
                    j--;
                }                while ((j > 0 && proxyPool[j - 1].minX > insert.minX));
                proxyPool[j] = insert;
            }
        }
    }
    
    private function insertionSortY(proxyPool : Array<Proxy>) : Void{
        if (numProxies == 1) 
            return;
        for (i in 1...numProxies){
            var insert : Proxy = proxyPool[i];
            if (proxyPool[i - 1].minY > insert.minY) {
                var j : Int = i;
                do{
                    proxyPool[j] = proxyPool[j - 1];
                    j--;
                }                while ((j > 0 && proxyPool[j - 1].minY > insert.minY));
                proxyPool[j] = insert;
            }
        }
    }
    
    private function insertionSortZ(proxyPool : Array<Proxy>) : Void{
        if (numProxies == 1) 
            return;
        for (i in 1...numProxies){
            var insert : Proxy = proxyPool[i];
            if (proxyPool[i - 1].minZ > insert.minZ) {
                var j : Int = i;
                do{
                    proxyPool[j] = proxyPool[j - 1];
                    j--;
                }                while ((j > 0 && proxyPool[j - 1].minZ > insert.minZ));
                proxyPool[j] = insert;
            }
        }
    }
}

