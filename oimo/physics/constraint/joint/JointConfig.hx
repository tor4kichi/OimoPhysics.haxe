
package oimo.physics.constraint.joint;


import oimo.math.Mat33;
import oimo.math.Vec3;
/**
	 * ジョイントの初期化時に使われる共通設定のクラスです。
	 * このクラスの変数はコピーして使われ、直接参照を持たれることはありません。
	 * @author saharan
	 */
class JointConfig
{
    /**
		 * 剛体1に対する初期状態での相対接続位置です。
		 */
    public var localRelativeAnchorPosition1 : Vec3;
    
    /**
		 * 剛体2に対する初期状態での相対接続位置です。
		 */
    public var localRelativeAnchorPosition2 : Vec3;
    
    /**
		 * 剛体1に対する初期状態での回転軸です。
		 * このオプションは一部のジョイントにおいてのみ有効です。
		 */
    public var localAxis1 : Vec3;
    
    /**
		 * 剛体2に対する初期状態での回転軸です。
		 * このオプションは一部のジョイントにおいてのみ有効です。
		 */
    public var localAxis2 : Vec3;
    
    /**
		 * 接続された剛体同士が衝突するかどうかを表します。
		 */
    public var allowCollision : Bool;
    
    /**
		 * 新しく JointConfig オブジェクトを作成します。
		 */
    public function new()
    {
        localRelativeAnchorPosition1 = new Vec3();
        localRelativeAnchorPosition2 = new Vec3();
        localAxis1 = new Vec3();
        localAxis2 = new Vec3();
    }
}

