
package oimo.physics.collision.narrow;


/**
	 * 接触点の識別のためのクラスです。
	 * @author saharan
	 */
class ContactID
{
    /**
		 * 識別に使われる一つ目のデータです。
		 */
    public var data1 : Int;
    
    /**
		 * 識別に使われる二つ目のデータです。
		 */
    public var data2 : Int;
    
    /**
		 * 識別データが反転しているかどうかを表します。
		 */
    public var flip : Bool;
    
    /**
		 * 新しく ContactID オブジェクトを作成します。
		 */
    public function new()
    {
        
    }
    
    public function equals(id : ContactID) : Bool{
        return flip == (id.flip) ? data1 == id.data1 && data2 == id.data2 : data2 == id.data1 && data1 == id.data2;
    }
}

