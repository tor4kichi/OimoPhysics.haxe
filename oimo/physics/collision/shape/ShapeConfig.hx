
package oimo.physics.collision.shape;


import oimo.math.Mat33;
import oimo.math.Vec3;
/**
	 * 形状の初期化時に使われる共通設定のクラスです。
	 * このクラスの変数はコピーして使われ、直接参照を持たれることはありません。
	 * @author saharan
	 */
class ShapeConfig
{
    /**
		 * 重心のワールド座標です。
		 */
    public var position : Vec3;
    
    /**
		 * 回転行列です。
		 * この値を変更すると、剛体ではなく剛体に含まれる形状の
		 * 相対的な角度が変更されることに注意してください。
		 */
    public var rotation : Mat33;
    
    /**
		 * 摩擦係数です。
		 */
    public var friction : Float;
    
    /**
		 * 反発係数です。
		 */
    public var restitution : Float;
    
    /**
		 * 密度です。
		 */
    public var density : Float;
    
    /**
		 * 新しく ShapeConfig オブジェクトを作成します。
		 */
    public function new()
    {
        position = new Vec3();
        rotation = new Mat33();
        friction = 0.5;
        restitution = 0.5;
        density = 1;
    }
}

