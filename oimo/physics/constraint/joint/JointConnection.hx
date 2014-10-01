package oimo.physics.constraint.joint;


import oimo.physics.dynamics.RigidBody;
/**
	 * ジョイントによる剛体の繋がりを扱うクラスです。
	 * @author saharan
	 */
class JointConnection
{
    /**
		 * 前のジョイントの繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var prev : JointConnection;
    
    /**
		 * 次のジョイントの繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var next : JointConnection;
    
    /**
		 * この繋がりによって繋がれている剛体です。
		 */
    public var connected : RigidBody;
    
    /**
		 * この繋がりの親となるジョイントです。
		 */
    public var parent : Joint;
    
    /**
		 * 新しく JointConnection オブジェクトを作成します。
		 * @param	parent この繋がりの親となるジョイント
		 */
    public function new(parent : Joint)
    {
        this.parent = parent;
    }
}

