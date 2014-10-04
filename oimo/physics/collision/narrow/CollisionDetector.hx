
package oimo.physics.collision.narrow;

import oimo.physics.collision.narrow.CollisionResult;
import openfl.errors.Error;

import oimo.physics.collision.shape.Shape;
/**
	 * より詳細な形状の衝突判定を行うクラスです。
	 * @author saharan
	 */
class CollisionDetector
{
    /**
		 * detectCollision 関数の引数に指定する形状の順序を、
		 * 反転して受け取るかどうかを表します。
		 */
    public var flip : Bool;
    
    /**
		 * 新しく CollisionDetector オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
    public function new()
    {
        
    }
    
    /**
		 * 二つの形状の詳細な衝突判定を行います。
		 * 形状の種類は指定された物である必要があります。
		 * @param	shape1 形状1
		 * @param	shape2 形状2
		 * @param	result 衝突結果の格納先
		 */
    public function detectCollision(shape1 : Shape, shape2 : Shape, result : CollisionResult) : Void{
        throw new Error("detectCollision 関数が継承されていません");
    }
}

