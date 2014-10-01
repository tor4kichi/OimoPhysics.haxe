package oimo.physics.collision.narrow;

import oimo.physics.collision.narrow.ContactInfo;

import oimo.math.Vec3;
import oimo.physics.collision.shape.Shape;
/**
	 * 衝突判定の結果を格納するクラスです。
	 * @author saharan
	 */
class CollisionResult
{
    /**
		 * 接触点情報の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var contactInfos : Array<ContactInfo>;
    
    /**
		 * 接触点情報の配列の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numContactInfos : Int;
    
    private var maxContactInfos : Int;
    
    /**
		 * 新しく CollisionResult オブジェクトを作成します。
		 * @param maxContactInfos 格納できる接触点情報の最大数です。
		 */
    public function new(maxContactInfos : Int)
    {
        this.maxContactInfos = maxContactInfos;
        contactInfos = new Array<ContactInfo>();
    }
    
    /**
		 * 衝突結果を追加します。
		 * @param	positionX ワールド座標系での衝突位置の x 値
		 * @param	positionY ワールド座標系での衝突位置の y 値
		 * @param	positionZ ワールド座標系での衝突位置の z 値
		 * @param	normalX ワールド座標系での法線方向の x 値
		 * @param	normalY ワールド座標系での法線方向の y 値
		 * @param	normalZ ワールド座標系での法線方向の z 値
		 * @param	overlap 二つの形状間の距離
		 * @param	shape1 形状1
		 * @param	shape2 形状2
		 * @param	data1 識別データ1
		 * @param	data2 識別データ2
		 * @param	flip 識別データの順序が反転しているかどうか
		 */
    public function addContactInfo(
            positionX : Float, positionY : Float, positionZ : Float,
            normalX : Float, normalY : Float, normalZ : Float,
            overlap : Float, shape1 : Shape, shape2 : Shape, data1 : Int, data2 : Int, flip : Bool) : Void{
        if (numContactInfos == maxContactInfos)             return;
        if (contactInfos[numContactInfos] == null) {
            contactInfos[numContactInfos] = new ContactInfo();
        }
        var c : ContactInfo = contactInfos[numContactInfos++];
        c.position.x = positionX;
        c.position.y = positionY;
        c.position.z = positionZ;
        c.normal.x = normalX;
        c.normal.y = normalY;
        c.normal.z = normalZ;
        c.overlap = overlap;
        c.shape1 = shape1;
        c.shape2 = shape2;
        c.id.data1 = data1;
        c.id.data2 = data2;
        c.id.flip = flip;
    }
}

