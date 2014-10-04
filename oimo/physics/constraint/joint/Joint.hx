
package oimo.physics.constraint.joint;

import oimo.physics.constraint.joint.JointConnection;

import oimo.math.Vec3;
import oimo.physics.constraint.Constraint;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;
/**
	 * 剛体同士を繋ぐジョイントのクラスです。
	 * @author saharan
	 */
class Joint extends Constraint
{
    /**
		 * 定義されていないことを表すジョイントの種類です。
		 */
    public static inline var JOINT_NULL : Int = 0x0;
    
    /**
		 * 距離ジョイント表すジョイントの種類です。
		 */
    public static inline var JOINT_DISTANCE : Int = 0x1;
    
    /**
		 * ボールジョイント表すジョイントの種類です。
		 */
    public static inline var JOINT_BALL : Int = 0x2;
    
    /**
		 * ヒンジジョイント表すジョイントの種類です。
		 */
    public static inline var JOINT_HINGE : Int = 0x3;
    
    /**
		 * ヒンジ2ジョイント表すジョイントの種類です。
		 */
    public static inline var JOINT_HINGE2 : Int = 0x4;
    
    /**
		 * ジョイントの種類を表します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var type : Int;
    
    /**
		 * 接続された剛体同士が衝突するかどうかを表します。
		 */
    public var allowCollision : Bool;
    
    /**
		 * 剛体1に対する初期状態での相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var localRelativeAnchorPosition1 : Vec3;
    
    /**
		 * 剛体2に対する初期状態での相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var localRelativeAnchorPosition2 : Vec3;
    
    /**
		 * 剛体1に対する相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var relativeAnchorPosition1 : Vec3;
    
    /**
		 * 剛体2に対する相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var relativeAnchorPosition2 : Vec3;
    
    /**
		 * 剛体1への接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var anchorPosition1 : Vec3;
    
    /**
		 * 剛体2への接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var anchorPosition2 : Vec3;
    
    /**
		 * 剛体1に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var connection1 : JointConnection;
    
    /**
		 * 剛体2に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var connection2 : JointConnection;
    
    /**
		 * 新しい Joint オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
    public function new()
    {
        super();
        localRelativeAnchorPosition1 = new Vec3();
        localRelativeAnchorPosition2 = new Vec3();
        relativeAnchorPosition1 = new Vec3();
        relativeAnchorPosition2 = new Vec3();
        anchorPosition1 = new Vec3();
        anchorPosition2 = new Vec3();
        connection1 = new JointConnection(this);
        connection2 = new JointConnection(this);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        super.preSolve(timeStep, invTimeStep);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        super.solve();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        super.postSolve();
    }
}

