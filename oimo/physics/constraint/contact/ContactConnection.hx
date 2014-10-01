package oimo.physics.constraint.contact;


import oimo.physics.collision.shape.Shape;
import oimo.physics.dynamics.RigidBody;
/**
	 * 接触点による形状と剛体の繋がりを扱うクラスです。
	 * @author saharan
	 */
class ContactConnection
{
    /**
		 * 前の接触点の繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var prev : ContactConnection;
    
    /**
		 * 次の接触点の繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var next : ContactConnection;
    
    /**
		 * この繋がりによって繋がれている形状です。
		 */
    public var connectedShape : Shape;
    
    /**
		 * この繋がりによって繋がれている剛体です。
		 */
    public var connectedBody : RigidBody;
    
    /**
		 * この繋がりの親となる接触点です。
		 */
    public var parent : Contact;
    
    /**
		 * 新しく ContactConnection オブジェクトを作成します。
		 * @param	parent この繋がりの親となる接触点
		 */
    public function new(parent : Contact)
    {
        this.parent = parent;
    }
}

