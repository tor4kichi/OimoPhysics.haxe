package oimo.physics.collision.broad;

import oimo.physics.collision.broad.Pair;
import oimo.physics.collision.broad.Proxy;
import openfl.errors.Error;

import oimo.physics.collision.shape.Shape;
import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConnection;
import oimo.physics.dynamics.RigidBody;
/**
	 * 広域衝突判定を行うクラスです。
	 * 広域衝突判定では、詳細な衝突判定の回数を削減するため、
	 * 詳細な形状の代わりに、近似された単純な形を用いて計算されます。
	 * 広域衝突判定の後、衝突の可能性がある形状のペアのみに、詳細な衝突判定が行われます。
	 * @author saharan
	 */
class BroadPhase
{
    /**
		 * プロキシが重なった形状のペアの配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var pairs : Array<Pair>;
    
    /**
		 * プロキシが重なった形状のペアの数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numPairs : Int;
    
    /**
		 * プロキシが重なった形状のペアを検索した回数です。
		 * アルゴリズムが総当りの場合、この数は形状の数を n とすると n * (n - 1) / 2 と表せます。
		 */
    public var numPairChecks : Int;
    
    private var bufferSize : Int;
    
    /**
		 * 新しく BroadPhase オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
    public function new()
    {
        bufferSize = 256;
        pairs = new Array<Pair>();
        for (i in 0...bufferSize){
            pairs[i] = new Pair();
        }
    }
    
    /**
		 * 判定対象のプロキシを追加します。
		 * @param	proxy 追加するプロキシ
		 */
    public function addProxy(proxy : Proxy) : Void{
        throw new Error("addProxy 関数が継承されていません");
    }
    
    /**
		 * 判定対象のプロキシを削除します。
		 * @param	proxy 削除するプロキシ
		 */
    public function removeProxy(proxy : Proxy) : Void{
        throw new Error("removeProxy 関数が継承されていません");
    }
    
    /**
		 * 指定された形状で構成されるペアが有効であるかどうかを判断します。
		 * @param	s1 形状1
		 * @param	s2 形状2
		 * @return ペアが有効なら true
		 */
    public function isAvailablePair(s1 : Shape, s2 : Shape) : Bool{
        var b1 : RigidBody = s1.parent;
        var b2 : RigidBody = s2.parent;
        if (
            b1 == b2 ||  // same parents  
            (b1.type == RigidBody.BODY_STATIC || b1.sleeping) &&
            (b2.type == RigidBody.BODY_STATIC || b2.sleeping)  // static (or sleeping) objects  
			) {
            return false;
        }
        var jc : JointConnection;
        if (b1.numJoints < b2.numJoints)             jc = b1.jointList
        else jc = b2.jointList;
        while (jc != null){
            var joint : Joint = jc.parent;
            if (
                !joint.allowCollision &&
                (joint.body1 == b1 && joint.body2 == b2 ||
                joint.body1 == b2 && joint.body2 == b1)) {
                return false;
            }
            jc = jc.next;
        }
        return true;
    }
    
    /**
		 * 衝突の可能性がある形状のペアを計算します。
		 */
    public function detectPairs() : Void{
        while (numPairs > 0){
            var pair : Pair = pairs[--numPairs];
            pair.shape1 = null;
            pair.shape2 = null;
        }
        collectPairs();
    }
    
    private function collectPairs() : Void{
        throw new Error("collectPairs 関数が継承されていません");
    }
    
    private function addPair(s1 : Shape, s2 : Shape) : Void{
        if (numPairs == bufferSize) {  // expand pair buffer  
            var newBufferSize : Int = bufferSize << 1;
            var newPairs : Array<Pair> = new Array<Pair>();
            for (i in 0...bufferSize){
                newPairs[i] = pairs[i];
            }
            for (i in bufferSize...newBufferSize){
                newPairs[i] = new Pair();
            }
            pairs = newPairs;
            bufferSize = newBufferSize;
        }
        var pair : Pair = pairs[numPairs++];
        pair.shape1 = s1;
        pair.shape2 = s2;
    }
}

