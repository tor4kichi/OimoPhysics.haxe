
package oimo.physics.collision.narrow;


import oimo.math.Vec3;
import oimo.physics.collision.shape.Shape;
import oimo.physics.dynamics.RigidBody;
/**
	 * 剛体同士の接触情報を保持するクラスです。
	 * @author saharan
	 */
class ContactInfo
{
    /**
		 * 接触地点のワールド座標です。
		 */
    public var position : Vec3;
    
    /**
		 * 接触面に対し垂直な法線ベクトルです。
		 */
    public var normal : Vec3;
    
    /**
		 * 形状間に発生した重なりの大きさです。
		 * 正確には形状間の距離を表すので、
		 * 重なりが発生した場合は、この変数は負の値を取ります。
		 */
    public var overlap : Float;
    
    /**
		 * 接触を起こした形状1です。
		 */
    public var shape1 : Shape;
    
    /**
		 * 接触を起こした形状2です。
		 */
    public var shape2 : Shape;
    
    /**
		 * 接触点の識別データです。
		 */
    public var id : ContactID;
    
    /**
		 * 新しく ContactInfo オブジェクトを作成します。
		 */
    public function new()
    {
        position = new Vec3();
        normal = new Vec3();
        id = new ContactID();
    }
}

