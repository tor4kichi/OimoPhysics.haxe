package oimo.physics.collision.broad;


import oimo.physics.collision.shape.Shape;
import oimo.physics.constraint.contact.Contact;
/**
	 * 衝突の可能性のある形状の対を扱うクラスです。
	 * @author saharan
	 */
class Pair
{
    /**
		 * 1つ目の形状です。
		 */
    public var shape1 : Shape;
    
    /**
		 * 2つ目の形状です。
		 */
    public var shape2 : Shape;
    
    /**
		 * 新しく Pair クラスを作成します。
		 */
    public function new()
    {
        
    }
}

