/* Copyright (c) 2012 EL-EMENT saharan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation  * files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy,  * modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package oimo.physics.collision.narrow;


import oimo.math.Vec3;
import oimo.physics.collision.shape.CylinderShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
/**
	 * 球体と円柱との詳細な衝突判定を行います。
	 * detectCollision 関数の引数に指定する形状は、
	 * コンストラクタで flip を true にしていない場合、
	 * 一つ目が球体、二つ目が円柱である必要があります。
	 * @author saharan
	 */
class SphereCylinderCollisionDetector extends CollisionDetector
{
    
    /**
		 * 新しく SphereCylinderCollisionDetector オブジェクトを作成します。
		 * @param	flip detectCollision 関数の引数に指定する形状の順序を、反転して受け取る場合は true
		 */
    public function new(flip : Bool)
    {
        super();
        this.flip = flip;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function detectCollision(shape1 : Shape, shape2 : Shape, result : CollisionResult) : Void{
        var s : SphereShape;
        var c : CylinderShape;
        if (flip) {
            s = try cast(shape2, SphereShape) catch(e:Dynamic) null;
            c = try cast(shape1, CylinderShape) catch(e:Dynamic) null;
        }
        else {
            s = try cast(shape1, SphereShape) catch(e:Dynamic) null;
            c = try cast(shape2, CylinderShape) catch(e:Dynamic) null;
        }
        var ps : Vec3 = s.position;
        var psx : Float = ps.x;
        var psy : Float = ps.y;
        var psz : Float = ps.z;
        var pc : Vec3 = c.position;
        var pcx : Float = pc.x;
        var pcy : Float = pc.y;
        var pcz : Float = pc.z;
        var dirx : Float = c.normalDirection.x;
        var diry : Float = c.normalDirection.y;
        var dirz : Float = c.normalDirection.z;
        var rads : Float = s.radius;
        var radc : Float = c.radius;
        var rad2 : Float = rads + radc;
        var halfh : Float = c.halfHeight;
        var dx : Float = psx - pcx;
        var dy : Float = psy - pcy;
        var dz : Float = psz - pcz;
        var dot : Float = dx * dirx + dy * diry + dz * dirz;
        if (dot < -halfh - rads || dot > halfh + rads)             return;
        var cx : Float = pcx + dot * dirx;
        var cy : Float = pcy + dot * diry;
        var cz : Float = pcz + dot * dirz;
        var d2x : Float = psx - cx;
        var d2y : Float = psy - cy;
        var d2z : Float = psz - cz;
        var len : Float = d2x * d2x + d2y * d2y + d2z * d2z;
        if (len > rad2 * rad2)             return;
        if (len > radc * radc) {
            len = radc / Math.sqrt(len);
            d2x *= len;
            d2y *= len;
            d2z *= len;
        }
        if (dot < -halfh)             dot = -halfh
        else if (dot > halfh)             dot = halfh;
        cx = pcx + dot * dirx + d2x;
        cy = pcy + dot * diry + d2y;
        cz = pcz + dot * dirz + d2z;
        dx = cx - psx;
        dy = cy - psy;
        dz = cz - psz;
        len = dx * dx + dy * dy + dz * dz;
        var invLen : Float;
        if (len > 0 && len < rads * rads) {
            len = Math.sqrt(len);
            invLen = 1 / len;
            dx *= invLen;
            dy *= invLen;
            dz *= invLen;
            result.addContactInfo(psx + dx * rads, psy + dy * rads, psz + dz * rads, dx, dy, dz, len - rads, s, c, 0, 0, false);
        }
    }
}

