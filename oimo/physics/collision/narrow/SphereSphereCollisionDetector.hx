
package oimo.physics.collision.narrow;


import oimo.math.Vec3;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
/**
	 * 球体同士の詳細な衝突判定を行います。
	 * detectCollision 関数の引数に指定する形状は、
	 * どちらも球体である必要があります。
	 * @author saharan
	 */
class SphereSphereCollisionDetector extends CollisionDetector
{
    
    /**
		 * 新しく SphereSphereCollisionDetector オブジェクトを作成します。
		 */
    public function new()
    {
        super();
        
    }
    
    /**
		 * @inheritDoc
		 */
    override public function detectCollision(shape1 : Shape, shape2 : Shape, result : CollisionResult) : Void{
        var s1 : SphereShape = try cast(shape1, SphereShape) catch(e:Dynamic) null;
        var s2 : SphereShape = try cast(shape2, SphereShape) catch(e:Dynamic) null;
        var p1 : Vec3 = s1.position;
        var p2 : Vec3 = s2.position;
        var dx : Float = p2.x - p1.x;
        var dy : Float = p2.y - p1.y;
        var dz : Float = p2.z - p1.z;
        var len : Float = dx * dx + dy * dy + dz * dz;
        var r1 : Float = s1.radius;
        var r2 : Float = s2.radius;
        var rad : Float = r1 + r2;
        if (len > 0 && len < rad * rad) {
            len = Math.sqrt(len);
            var invLen : Float = 1 / len;
            dx *= invLen;
            dy *= invLen;
            dz *= invLen;
            result.addContactInfo(p1.x + dx * r1, p1.y + dy * r1, p1.z + dz * r1, dx, dy, dz, len - rad, s1, s2, 0, 0, false);
        }
    }
}

