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
package oimo.math;


/**
	 * クォータニオンを扱うクラスです。
	 * クォータニオンは右手系のクォータニオンとして扱われます。
	 * オブジェクトの不要な作成を避けるため、
	 * 関数ではほとんどの演算結果は自身のオブジェクトに格納されます。
	 * @author saharan
	 */
class Quat
{
    /**
		 * スカラー成分です。
		 */
    public var s : Float;
    /**
		 * x 成分です。
		 */
    public var x : Float;
    /**
		 * y 成分です。
		 */
    public var y : Float;
    /**
		 * z 成分です。
		 */
    public var z : Float;
    
    /**
		 * 新しく Quat オブジェクトを作成します。
		 * 引数を指定しない場合は、単位クォータニオンで初期化されます。
		 * @param	s スカラー成分
		 * @param	x x 成分
		 * @param	y y 成分
		 * @param	z z 成分
		 */
    public function new(s : Float = 1, x : Float = 0, y : Float = 0, z : Float = 0)
    {
        this.s = s;
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    /**
		 * このクォータニオンを指定された値で初期化します。
		 * 引数を指定しない場合は、単位クォータニオンで初期化されます。
		 * @param	s スカラー成分
		 * @param	x x 成分
		 * @param	y y 成分
		 * @param	z z 成分
		 */
    public function init(s : Float = 1, x : Float = 0, y : Float = 0, z : Float = 0) : Quat{
        this.s = s;
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    
    /**
		 * このクォータニオンを q1 と q2 を加算したクォータニオンに設定します。
		 * @param	q1 クォータニオン1
		 * @param	q2 クォータニオン2
		 * @return このオブジェクト
		 */
    public function add(q1 : Quat, q2 : Quat) : Quat{
        s = q1.s + q2.s;
        x = q1.x + q2.x;
        y = q1.y + q2.y;
        z = q1.z + q2.z;
        return this;
    }
    
    /**
		 * このクォータニオンを q1 から q2 を減算したクォータニオンに設定します。
		 * @param	q1 クォータニオン1
		 * @param	q2 クォータニオン2
		 * @return このオブジェクト
		 */
    public function sub(q1 : Quat, q2 : Quat) : Quat{
        s = q1.s - q2.s;
        x = q1.x - q2.x;
        y = q1.y - q2.y;
        z = q1.z - q2.z;
        return this;
    }
    
    /**
		 * このクォータニオンを q を s 倍に拡張したクォータニオンに設定します。
		 * @param	q クォータニオン
		 * @param	s スカラー
		 * @return このオブジェクト
		 */
    public function scale(q : Quat, s : Float) : Quat{
        this.s = q.s * s;
        x = q.x * s;
        y = q.y * s;
        z = q.z * s;
        return this;
    }
    
    /**
		 * このクォータニオンを q1 と q2 を合成したクォータニオンに設定します。
		 * @param	q1 クォータニオン1
		 * @param	q2 クォータニオン2
		 * @return このオブジェクト
		 */
    public function mul(q1 : Quat, q2 : Quat) : Quat{
        var s : Float = q1.s * q2.s - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        var x : Float = q1.s * q2.x + q1.x * q2.s + q1.y * q2.z - q1.z * q2.y;
        var y : Float = q1.s * q2.y - q1.x * q2.z + q1.y * q2.s + q1.z * q2.x;
        var z : Float = q1.s * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.s;
        this.s = s;
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    
    /**
		 * このクォータニオンを q を正規化したクォータニオンに設定します。
		 * @param	q クォータニオン
		 * @return このオブジェクト
		 */
    public function normalize(q : Quat) : Quat{
        var len : Float = Math.sqrt(q.s * q.s + q.x * q.x + q.y * q.y + q.z * q.z);
        if (len > 0)             len = 1 / len;
        s = q.s * len;
        x = q.x * len;
        y = q.y * len;
        z = q.z * len;
        return this;
    }
    
    /**
		 * このクォータニオンを q を反転させたクォータニオンに設定します。
		 * @param	q クォータニオン
		 * @return このオブジェクト
		 */
    public function invert(q : Quat) : Quat{
        s = -q.s;
        x = -q.x;
        y = -q.y;
        z = -q.z;
        return this;
    }
    
    /**
		 * このクォータニオンの大きさを返します。
		 * @return 大きさ
		 */
    public function length() : Float{
        return Math.sqrt(s * s + x * x + y * y + z * z);
    }
    
    /**
		 * このクォータニオンの値を q からコピーします。
		 * @param	q クォータニオン
		 * @return このオブジェクト
		 */
    public function copy(q : Quat) : Quat{
        s = q.s;
        x = q.x;
        y = q.y;
        z = q.z;
        return this;
    }
    
    /**
		 * この Quat オブジェクトを複製します。
		 * @return 複製された Quat オブジェクト
		 */
    public function clone(q : Quat) : Quat{
        return new Quat(s, x, y, z);
    }
    
    /**
		 * このクォータニオンの文字列表現を返します。
		 * @return このクォータニオンを表す文字列
		 */
    public function toString() : String{
        return "Quat[" + Std.string(s) + ", (" + Std.string(x) + ", " + Std.string(y) + ", " + Std.string(z) + ")]";
    }
}

