
package oimo.physics.collision.narrow;


import oimo.math.Vec3;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
/**
	 * 球体と箱との詳細な衝突判定を行います。
	 * detectCollision 関数の引数に指定する形状は、
	 * コンストラクタで flip を true にしていない場合、
	 * 一つ目が球体、二つ目が箱である必要があります。
	 * @author saharan
	 */
class SphereBoxCollisionDetector extends CollisionDetector
{
    
    /**
		 * 新しく SphereBoxCollisionDetector オブジェクトを作成します。
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
        var b : BoxShape;
        if (flip) {
            s = try cast(shape2, SphereShape) catch(e:Dynamic) null;
            b = try cast(shape1, BoxShape) catch(e:Dynamic) null;
        }
        else {
            s = try cast(shape1, SphereShape) catch(e:Dynamic) null;
            b = try cast(shape2, BoxShape) catch(e:Dynamic) null;
        }
        var ps : Vec3 = s.position;
        var psx : Float = ps.x;
        var psy : Float = ps.y;
        var psz : Float = ps.z;
        var pb : Vec3 = b.position;
        var pbx : Float = pb.x;
        var pby : Float = pb.y;
        var pbz : Float = pb.z;
        var rad : Float = s.radius;
        // normal
        var nw : Vec3 = b.normalDirectionWidth;
        var nh : Vec3 = b.normalDirectionHeight;
        var nd : Vec3 = b.normalDirectionDepth;
        // half
        var hw : Float = b.halfWidth;
        var hh : Float = b.halfHeight;
        var hd : Float = b.halfDepth;
        // diff
        var dx : Float = psx - pbx;
        var dy : Float = psy - pby;
        var dz : Float = psz - pbz;
        // shadow
        var sx : Float = nw.x * dx + nw.y * dy + nw.z * dz;
        var sy : Float = nh.x * dx + nh.y * dy + nh.z * dz;
        var sz : Float = nd.x * dx + nd.y * dy + nd.z * dz;
        // closest
        var cx : Float;
        var cy : Float;
        var cz : Float;
        var len : Float;
        var invLen : Float;
        var c : ContactInfo;
        var overlap : Int = 0;
        if (sx > hw) {
            sx = hw;
        }
        else if (sx < -hw) {
            sx = -hw;
        }
        else {
            overlap = 1;
        }
        if (sy > hh) {
            sy = hh;
        }
        else if (sy < -hh) {
            sy = -hh;
        }
        else {
            overlap |= 2;
        }
        if (sz > hd) {
            sz = hd;
        }
        else if (sz < -hd) {
            sz = -hd;
        }
        else {
            overlap |= 4;
        }
        if (overlap == 7) {
            // center of sphere is in the box
            if (sx < 0) {
                dx = hw + sx;
            }
            else {
                dx = hw - sx;
            }
            if (sy < 0) {
                dy = hh + sy;
            }
            else {
                dy = hh - sy;
            }
            if (sz < 0) {
                dz = hd + sz;
            }
            else {
                dz = hd - sz;
            }
            if (dx < dy) {
                if (dx < dz) {
                    // x
                    len = dx - hw;
                    if (sx < 0) {
                        sx = -hw;
                        dx = nw.x;
                        dy = nw.y;
                        dz = nw.z;
                    }
                    else {
                        sx = hw;
                        dx = -nw.x;
                        dy = -nw.y;
                        dz = -nw.z;
                    }
                }
                else {
                    // z
                    len = dz - hd;
                    if (sz < 0) {
                        sz = -hd;
                        dx = nd.x;
                        dy = nd.y;
                        dz = nd.z;
                    }
                    else {
                        sz = hd;
                        dx = -nd.x;
                        dy = -nd.y;
                        dz = -nd.z;
                    }
                }
            }
            else {
                if (dy < dz) {
                    // y
                    len = dy - hh;
                    if (sy < 0) {
                        sy = -hh;
                        dx = nh.x;
                        dy = nh.y;
                        dz = nh.z;
                    }
                    else {
                        sy = hh;
                        dx = -nh.x;
                        dy = -nh.y;
                        dz = -nh.z;
                    }
                }
                else {
                    // z
                    len = dz - hd;
                    if (sz < 0) {
                        sz = -hd;
                        dx = nd.x;
                        dy = nd.y;
                        dz = nd.z;
                    }
                    else {
                        sz = hd;
                        dx = -nd.x;
                        dy = -nd.y;
                        dz = -nd.z;
                    }
                }
            }
            cx = pbx + sx * nw.x + sy * nh.x + sz * nd.x;
            cy = pby + sx * nw.y + sy * nh.y + sz * nd.y;
            cz = pbz + sx * nw.z + sy * nh.z + sz * nd.z;
            result.addContactInfo(psx + rad * dx, psy + rad * dy, psz + rad * dz, dx, dy, dz, len, s, b, 0, 0, false);
        }
        else {
            // closest
            cx = pbx + sx * nw.x + sy * nh.x + sz * nd.x;
            cy = pby + sx * nw.y + sy * nh.y + sz * nd.y;
            cz = pbz + sx * nw.z + sy * nh.z + sz * nd.z;
            dx = cx - ps.x;
            dy = cy - ps.y;
            dz = cz - ps.z;
            len = dx * dx + dy * dy + dz * dz;
            if (len > 0 && len < rad * rad) {
                len = Math.sqrt(len);
                invLen = 1 / len;
                dx *= invLen;
                dy *= invLen;
                dz *= invLen;
                result.addContactInfo(psx + rad * dx, psy + rad * dy, psz + rad * dz, dx, dy, dz, len, s, b, 0, 0, false);
            }
        }
    }
}

