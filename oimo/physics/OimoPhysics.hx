
package oimo.physics;

import openfl.errors.Error;

/**
	 * OimoPhysics の情報のクラスです。
	 * @author saharan
	 */
@:final class OimoPhysics
{
    /**
		 * OimoPhysics のバージョンです。
		 */
    public static inline var VERSION : String = "1.0.0";
    
    /**
		 * OimoPhysics の説明です。
		 */
    public static var DESCRIPTION : String = "OimoPhysics " + VERSION + " (c) 2012 EL-EMENT saharan";
    
    /**
		 * OimoPhysics オブジェクトを作成することはできません。
		 */
    public function new()
    {
        throw new Error("OimoPhysics オブジェクトを作成することはできません");
    }
}

