
package oimo.math;

import oimo.math.Quat;

/**
	 * 4行4列の要素を持つ行列を扱うクラスです。
	 * この行列は、ある三次元座標系から別の三次元座標系への、平行移動を含む完全な変換をサポートします。
	 * 行列は右手系の行列として扱われます。
	 * オブジェクトの不要な作成を避けるため、
	 * 関数ではほとんどの演算結果は自身のオブジェクトに格納されます。
	 * @author saharan
	 */
class Mat44
{
    /**
		 * 1行1列目の要素です。
		 */
    public var e00 : Float;
    /**
		 * 1行2列目の要素です。
		 */
    public var e01 : Float;
    /**
		 * 1行3列目の要素です。
		 */
    public var e02 : Float;
    /**
		 * 1行4列目の要素です。
		 */
    public var e03 : Float;
    /**
		 * 2行1列目の要素です。
		 */
    public var e10 : Float;
    /**
		 * 2行2列目の要素です。
		 */
    public var e11 : Float;
    /**
		 * 2行3列目の要素です。
		 */
    public var e12 : Float;
    /**
		 * 2行4列目の要素です。
		 */
    public var e13 : Float;
    /**
		 * 3行1列目の要素です。
		 */
    public var e20 : Float;
    /**
		 * 3行2列目の要素です。
		 */
    public var e21 : Float;
    /**
		 * 3行3列目の要素です。
		 */
    public var e22 : Float;
    /**
		 * 3行4列目の要素です。
		 */
    public var e23 : Float;
    /**
		 * 4行1列目の要素です。
		 */
    public var e30 : Float;
    /**
		 * 4行2列目の要素です。
		 */
    public var e31 : Float;
    /**
		 * 4行3列目の要素です。
		 */
    public var e32 : Float;
    /**
		 * 4行4列目の要素です。
		 */
    public var e33 : Float;
    
    /**
		 * 新しく Mat44 オブジェクトを作成します。
		 * 引数を指定しない場合は、単位行列で初期化されます。
		 * @param	e00 設定する1行1列目の要素
		 * @param	e01 設定する1行2列目の要素
		 * @param	e02 設定する1行3列目の要素
		 * @param	e03 設定する1行4列目の要素
		 * @param	e10 設定する2行1列目の要素
		 * @param	e11 設定する2行2列目の要素
		 * @param	e12 設定する2行3列目の要素
		 * @param	e13 設定する2行4列目の要素
		 * @param	e20 設定する3行1列目の要素
		 * @param	e21 設定する3行2列目の要素
		 * @param	e22 設定する3行3列目の要素
		 * @param	e23 設定する3行4列目の要素
		 * @param	e30 設定する4行1列目の要素
		 * @param	e31 設定する4行2列目の要素
		 * @param	e32 設定する4行3列目の要素
		 * @param	e33 設定する4行4列目の要素
		 */
    public function new(
            e00 : Float = 1, e01 : Float = 0, e02 : Float = 0, e03 : Float = 0,
            e10 : Float = 0, e11 : Float = 1, e12 : Float = 0, e13 : Float = 0,
            e20 : Float = 0, e21 : Float = 0, e22 : Float = 1, e23 : Float = 0,
            e30 : Float = 0, e31 : Float = 0, e32 : Float = 0, e33 : Float = 1)
    {
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        this.e33 = e33;
    }
    
    /**
		 * この行列を指定された値で初期化します。
		 * 引数を指定しない場合は、単位行列で初期化されます。
		 * @param	e00 設定する1行1列目の要素
		 * @param	e01 設定する1行2列目の要素
		 * @param	e02 設定する1行3列目の要素
		 * @param	e03 設定する1行4列目の要素
		 * @param	e10 設定する2行1列目の要素
		 * @param	e11 設定する2行2列目の要素
		 * @param	e12 設定する2行3列目の要素
		 * @param	e13 設定する2行4列目の要素
		 * @param	e20 設定する3行1列目の要素
		 * @param	e21 設定する3行2列目の要素
		 * @param	e22 設定する3行3列目の要素
		 * @param	e23 設定する3行4列目の要素
		 * @param	e30 設定する4行1列目の要素
		 * @param	e31 設定する4行2列目の要素
		 * @param	e32 設定する4行3列目の要素
		 * @param	e33 設定する4行4列目の要素
		 * @return このオブジェクト
		 */
    public function init(
            e00 : Float = 1, e01 : Float = 0, e02 : Float = 0, e03 : Float = 0,
            e10 : Float = 0, e11 : Float = 1, e12 : Float = 0, e13 : Float = 0,
            e20 : Float = 0, e21 : Float = 0, e22 : Float = 1, e23 : Float = 0,
            e30 : Float = 0, e31 : Float = 0, e32 : Float = 0, e33 : Float = 1) : Mat44{
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        this.e33 = e33;
        return this;
    }
    
    /**
		 * この行列を m1 と m2 を加算した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
    public function add(m1 : Mat44, m2 : Mat44) : Mat44{
        e00 = m1.e00 + m2.e00;
        e01 = m1.e01 + m2.e01;
        e02 = m1.e02 + m2.e02;
        e03 = m1.e03 + m2.e03;
        e10 = m1.e10 + m2.e10;
        e11 = m1.e11 + m2.e11;
        e12 = m1.e12 + m2.e12;
        e13 = m1.e13 + m2.e13;
        e20 = m1.e20 + m2.e20;
        e21 = m1.e21 + m2.e21;
        e22 = m1.e22 + m2.e22;
        e23 = m1.e23 + m2.e23;
        e30 = m1.e30 + m2.e30;
        e31 = m1.e31 + m2.e31;
        e32 = m1.e32 + m2.e32;
        e33 = m1.e33 + m2.e33;
        return this;
    }
    
    /**
		 * この行列を m1 から m2 を減算した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
    public function sub(m1 : Mat44, m2 : Mat44) : Mat44{
        e00 = m1.e00 - m2.e00;
        e01 = m1.e01 - m2.e01;
        e02 = m1.e02 - m2.e02;
        e03 = m1.e03 - m2.e03;
        e10 = m1.e10 - m2.e10;
        e11 = m1.e11 - m2.e11;
        e12 = m1.e12 - m2.e12;
        e13 = m1.e13 - m2.e13;
        e20 = m1.e20 - m2.e20;
        e21 = m1.e21 - m2.e21;
        e22 = m1.e22 - m2.e22;
        e23 = m1.e23 - m2.e23;
        e30 = m1.e30 - m2.e30;
        e31 = m1.e31 - m2.e31;
        e32 = m1.e32 - m2.e32;
        e33 = m1.e33 - m2.e33;
        return this;
    }
    
    /**
		 * この行列を m を s 倍に拡張した行列に設定します。
		 * @param	m 行列
		 * @param	s スカラー
		 * @return このオブジェクト
		 */
    public function scale(m : Mat44, s : Float) : Mat44{
        e00 = m.e00 * s;
        e01 = m.e01 * s;
        e02 = m.e02 * s;
        e03 = m.e03 * s;
        e10 = m.e10 * s;
        e11 = m.e11 * s;
        e12 = m.e12 * s;
        e13 = m.e13 * s;
        e20 = m.e20 * s;
        e21 = m.e21 * s;
        e22 = m.e22 * s;
        e23 = m.e23 * s;
        e30 = m.e30 * s;
        e31 = m.e31 * s;
        e32 = m.e32 * s;
        e33 = m.e33 * s;
        return this;
    }
    
    /**
		 * この行列を m1 と m2 を合成した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
    public function mul(m1 : Mat44, m2 : Mat44) : Mat44{
        var e00 : Float = m1.e00 * m2.e00 + m1.e01 * m2.e10 + m1.e02 * m2.e20 + m1.e03 * m2.e30;
        var e01 : Float = m1.e00 * m2.e01 + m1.e01 * m2.e11 + m1.e02 * m2.e21 + m1.e03 * m2.e31;
        var e02 : Float = m1.e00 * m2.e02 + m1.e01 * m2.e12 + m1.e02 * m2.e22 + m1.e03 * m2.e32;
        var e03 : Float = m1.e00 * m2.e03 + m1.e01 * m2.e13 + m1.e02 * m2.e23 + m1.e03 * m2.e33;
        var e10 : Float = m1.e10 * m2.e00 + m1.e11 * m2.e10 + m1.e12 * m2.e20 + m1.e13 * m2.e30;
        var e11 : Float = m1.e10 * m2.e01 + m1.e11 * m2.e11 + m1.e12 * m2.e21 + m1.e13 * m2.e31;
        var e12 : Float = m1.e10 * m2.e02 + m1.e11 * m2.e12 + m1.e12 * m2.e22 + m1.e13 * m2.e32;
        var e13 : Float = m1.e10 * m2.e03 + m1.e11 * m2.e13 + m1.e12 * m2.e23 + m1.e13 * m2.e33;
        var e20 : Float = m1.e20 * m2.e00 + m1.e21 * m2.e10 + m1.e22 * m2.e20 + m1.e23 * m2.e30;
        var e21 : Float = m1.e20 * m2.e01 + m1.e21 * m2.e11 + m1.e22 * m2.e21 + m1.e23 * m2.e31;
        var e22 : Float = m1.e20 * m2.e02 + m1.e21 * m2.e12 + m1.e22 * m2.e22 + m1.e23 * m2.e32;
        var e23 : Float = m1.e20 * m2.e03 + m1.e21 * m2.e13 + m1.e22 * m2.e23 + m1.e23 * m2.e33;
        var e30 : Float = m1.e30 * m2.e00 + m1.e31 * m2.e10 + m1.e32 * m2.e20 + m1.e33 * m2.e30;
        var e31 : Float = m1.e30 * m2.e01 + m1.e31 * m2.e11 + m1.e32 * m2.e21 + m1.e33 * m2.e31;
        var e32 : Float = m1.e30 * m2.e02 + m1.e31 * m2.e12 + m1.e32 * m2.e22 + m1.e33 * m2.e32;
        var e33 : Float = m1.e30 * m2.e03 + m1.e31 * m2.e13 + m1.e32 * m2.e23 + m1.e33 * m2.e33;
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        this.e33 = e33;
        return this;
    }
    
    /**
		 * この行列を m と拡大縮小行列を合成したものに設定します。
		 * @param	m 行列
		 * @param	sx x 方向の拡大率
		 * @param	sy　y 方向の拡大率
		 * @param	sz　z 方向の拡大率
		 * @param	prepend 合成順序を逆にする場合は true
		 * @return このオブジェクト
		 */
    public function mulScale(m : Mat44, sx : Float, sy : Float, sz : Float, prepend : Bool = false) : Mat44{
        var e00 : Float;
        var e01 : Float;
        var e02 : Float;
        var e03 : Float;
        var e10 : Float;
        var e11 : Float;
        var e12 : Float;
        var e13 : Float;
        var e20 : Float;
        var e21 : Float;
        var e22 : Float;
        var e23 : Float;
        var e30 : Float;
        var e31 : Float;
        var e32 : Float;
        var e33 : Float;
        if (prepend) {
            e00 = sx * m.e00;
            e01 = sx * m.e01;
            e02 = sx * m.e02;
            e03 = sx * m.e03;
            e10 = sy * m.e10;
            e11 = sy * m.e11;
            e12 = sy * m.e12;
            e13 = sy * m.e13;
            e20 = sz * m.e20;
            e21 = sz * m.e21;
            e22 = sz * m.e22;
            e23 = sz * m.e23;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        else {
            e00 = m.e00 * sx;
            e01 = m.e01 * sy;
            e02 = m.e02 * sz;
            e03 = m.e03;
            e10 = m.e10 * sx;
            e11 = m.e11 * sy;
            e12 = m.e12 * sz;
            e13 = m.e13;
            e20 = m.e20 * sx;
            e21 = m.e21 * sy;
            e22 = m.e22 * sz;
            e23 = m.e23;
            e30 = m.e30 * sx;
            e31 = m.e31 * sy;
            e32 = m.e32 * sz;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        return this;
    }
    
    /**
		 * この行列を m と回転行列を合成したものに設定します。
		 * @param	m 行列
		 * @param	rad ラジアンでの回転角度
		 * @param	ax 回転軸の x 成分
		 * @param	ay 回転軸の y 成分
		 * @param	az 回転軸の z 成分
		 * @param	prepend 合成順序を逆にする場合は true
		 * @return このオブジェクト
		 */
    public function mulRotate(m : Mat44, rad : Float, ax : Float, ay : Float, az : Float, prepend : Bool = false) : Mat44{
        var s : Float = Math.sin(rad);
        var c : Float = Math.cos(rad);
        var c1 : Float = 1 - c;
        var r00 : Float = ax * ax * c1 + c;
        var r01 : Float = ax * ay * c1 - az * s;
        var r02 : Float = ax * az * c1 + ay * s;
        var r10 : Float = ay * ax * c1 + az * s;
        var r11 : Float = ay * ay * c1 + c;
        var r12 : Float = ay * az * c1 - ax * s;
        var r20 : Float = az * ax * c1 - ay * s;
        var r21 : Float = az * ay * c1 + ax * s;
        var r22 : Float = az * az * c1 + c;
        var e00 : Float;
        var e01 : Float;
        var e02 : Float;
        var e03 : Float;
        var e10 : Float;
        var e11 : Float;
        var e12 : Float;
        var e13 : Float;
        var e20 : Float;
        var e21 : Float;
        var e22 : Float;
        var e23 : Float;
        var e30 : Float;
        var e31 : Float;
        var e32 : Float;
        var e33 : Float;
        if (prepend) {
            e00 = r00 * m.e00 + r01 * m.e10 + r02 * m.e20;
            e01 = r00 * m.e01 + r01 * m.e11 + r02 * m.e21;
            e02 = r00 * m.e02 + r01 * m.e12 + r02 * m.e22;
            e03 = r00 * m.e03 + r01 * m.e13 + r02 * m.e23;
            e10 = r10 * m.e00 + r11 * m.e10 + r12 * m.e20;
            e11 = r10 * m.e01 + r11 * m.e11 + r12 * m.e21;
            e12 = r10 * m.e02 + r11 * m.e12 + r12 * m.e22;
            e13 = r10 * m.e03 + r11 * m.e13 + r12 * m.e23;
            e20 = r20 * m.e00 + r21 * m.e10 + r22 * m.e20;
            e21 = r20 * m.e01 + r21 * m.e11 + r22 * m.e21;
            e22 = r20 * m.e02 + r21 * m.e12 + r22 * m.e22;
            e23 = r20 * m.e03 + r21 * m.e13 + r22 * m.e23;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        else {
            e00 = m.e00 * r00 + m.e01 * r10 + m.e02 * r20;
            e01 = m.e00 * r01 + m.e01 * r11 + m.e02 * r21;
            e02 = m.e00 * r02 + m.e01 * r12 + m.e02 * r22;
            e03 = m.e03;
            e10 = m.e10 * r00 + m.e11 * r10 + m.e12 * r20;
            e11 = m.e10 * r01 + m.e11 * r11 + m.e12 * r21;
            e12 = m.e10 * r02 + m.e11 * r12 + m.e12 * r22;
            e13 = m.e13;
            e20 = m.e20 * r00 + m.e21 * r10 + m.e22 * r20;
            e21 = m.e20 * r01 + m.e21 * r11 + m.e22 * r21;
            e22 = m.e20 * r02 + m.e21 * r12 + m.e22 * r22;
            e23 = m.e23;
            e30 = m.e30 * r00 + m.e31 * r10 + m.e32 * r20;
            e31 = m.e30 * r01 + m.e31 * r11 + m.e32 * r21;
            e32 = m.e30 * r02 + m.e31 * r12 + m.e32 * r22;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        return this;
    }
    
    /**
		 * この行列を m と平行移動行列を合成したものに設定します。
		 * @param	m 行列
		 * @param	tx x 方向の平行移動成分
		 * @param	ty y 方向の平行移動成分
		 * @param	tz z 方向の平行移動成分
		 * @param	prepend 合成順序を逆にする場合は true
		 * @return このオブジェクト
		 */
    public function mulTranslate(m : Mat44, tx : Float, ty : Float, tz : Float, prepend : Bool = false) : Mat44{
        var e00 : Float;
        var e01 : Float;
        var e02 : Float;
        var e03 : Float;
        var e10 : Float;
        var e11 : Float;
        var e12 : Float;
        var e13 : Float;
        var e20 : Float;
        var e21 : Float;
        var e22 : Float;
        var e23 : Float;
        var e30 : Float;
        var e31 : Float;
        var e32 : Float;
        var e33 : Float;
        if (prepend) {
            e00 = m.e00 + tx * m.e30;
            e01 = m.e01 + tx * m.e31;
            e02 = m.e02 + tx * m.e32;
            e03 = m.e03 + tx * m.e33;
            e10 = m.e10 + ty * m.e30;
            e11 = m.e11 + ty * m.e31;
            e12 = m.e12 + ty * m.e32;
            e13 = m.e13 + ty * m.e33;
            e20 = m.e20 + tz * m.e30;
            e21 = m.e21 + tz * m.e31;
            e22 = m.e22 + tz * m.e32;
            e23 = m.e23 + tz * m.e33;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        else {
            e00 = m.e00;
            e01 = m.e01;
            e02 = m.e02;
            e03 = m.e00 * tx + m.e01 * ty + m.e02 * tz + m.e03;
            e10 = m.e10;
            e11 = m.e11;
            e12 = m.e12;
            e13 = m.e10 * tx + m.e11 * ty + m.e12 * tz + m.e13;
            e20 = m.e20;
            e21 = m.e21;
            e22 = m.e22;
            e23 = m.e20 * tx + m.e21 * ty + m.e22 * tz + m.e23;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e30 * tx + m.e31 * ty + m.e32 * tz + m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        return this;
    }
    
    /**
		 * この行列を m の転置行列に設定します。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
    public function transpose(m : Mat44) : Mat44{
        var e01 : Float = m.e10;
        var e02 : Float = m.e20;
        var e03 : Float = m.e30;
        var e10 : Float = m.e01;
        var e12 : Float = m.e21;
        var e13 : Float = m.e31;
        var e20 : Float = m.e02;
        var e21 : Float = m.e12;
        var e23 : Float = m.e32;
        var e30 : Float = m.e03;
        var e31 : Float = m.e13;
        var e32 : Float = m.e23;
        e00 = m.e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        e11 = m.e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        e22 = m.e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        e33 = m.e33;
        return this;
    }
    
    /**
		 * この行列を q で表される回転行列に設定します。
		 * @param	q クォータニオン
		 * @return このオブジェクト
		 */
    public function setQuat(q : Quat) : Mat44{
        var x2 : Float = 2 * q.x;
        var y2 : Float = 2 * q.y;
        var z2 : Float = 2 * q.z;
        var xx : Float = q.x * x2;
        var yy : Float = q.y * y2;
        var zz : Float = q.z * z2;
        var xy : Float = q.x * y2;
        var yz : Float = q.y * z2;
        var xz : Float = q.x * z2;
        var sx : Float = q.s * x2;
        var sy : Float = q.s * y2;
        var sz : Float = q.s * z2;
        e00 = 1 - yy - zz;
        e01 = xy - sz;
        e02 = xz + sy;
        e03 = 0;
        e10 = xy + sz;
        e11 = 1 - xx - zz;
        e12 = yz - sx;
        e13 = 0;
        e20 = xz - sy;
        e21 = yz + sx;
        e22 = 1 - xx - yy;
        e23 = 0;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
		 * この行列を m の逆行列に設定します。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
    public function invert(m : Mat44) : Mat44{
        var e1021_1120 : Float = m.e10 * m.e21 - m.e11 * m.e20;
        var e1022_1220 : Float = m.e10 * m.e22 - m.e12 * m.e20;
        var e1023_1320 : Float = m.e10 * m.e23 - m.e13 * m.e20;
        var e1031_1130 : Float = m.e10 * m.e31 - m.e11 * m.e30;
        var e1032_1230 : Float = m.e10 * m.e32 - m.e12 * m.e30;
        var e1033_1330 : Float = m.e10 * m.e33 - m.e13 * m.e30;
        var e1122_1221 : Float = m.e11 * m.e22 - m.e12 * m.e21;
        var e1123_1321 : Float = m.e11 * m.e23 - m.e13 * m.e21;
        var e1132_1231 : Float = m.e11 * m.e32 - m.e12 * m.e31;
        var e1133_1331 : Float = m.e11 * m.e33 - m.e13 * m.e31;
        var e1220_2022 : Float = m.e12 * m.e20 - m.e20 * m.e22;
        var e1223_1322 : Float = m.e12 * m.e23 - m.e13 * m.e22;
        var e1223_2223 : Float = m.e12 * m.e33 - m.e22 * m.e23;
        var e1233_1332 : Float = m.e12 * m.e33 - m.e13 * m.e32;
        var e2031_2130 : Float = m.e20 * m.e31 - m.e21 * m.e30;
        var e2032_2033 : Float = m.e20 * m.e32 - m.e20 * m.e33;
        var e2032_2230 : Float = m.e20 * m.e32 - m.e22 * m.e30;
        var e2033_2330 : Float = m.e20 * m.e33 - m.e23 * m.e30;
        var e2132_2231 : Float = m.e21 * m.e32 - m.e22 * m.e31;
        var e2133_2331 : Float = m.e21 * m.e33 - m.e23 * m.e31;
        var e2230_2330 : Float = m.e22 * m.e30 - m.e23 * m.e30;
        var e2233_2332 : Float = m.e22 * m.e33 - m.e23 * m.e32;
        var det : Float = 
        m.e00 * (m.e11 * e2233_2332 - m.e12 * e2133_2331 + m.e13 * e2132_2231) +
        m.e01 * (-m.e10 * e2233_2332 - m.e12 * e2032_2033 + m.e13 * e2230_2330) +
        m.e02 * (m.e10 * e2133_2331 - m.e11 * e2033_2330 + m.e13 * e2031_2130) +
        m.e03 * (-m.e10 * e2132_2231 + m.e11 * e2032_2230 - m.e12 * e2031_2130);
        if (det != 0)             det = 1 / det;
        var t00 : Float = m.e11 * e2233_2332 - m.e12 * e2133_2331 + m.e13 * e2132_2231;
        var t01 : Float = -m.e01 * e2233_2332 + m.e02 * e2133_2331 - m.e03 * e2132_2231;
        var t02 : Float = m.e01 * e1233_1332 - m.e02 * e1133_1331 + m.e03 * e1132_1231;
        var t03 : Float = -m.e01 * e1223_2223 + m.e02 * e1123_1321 - m.e03 * e1122_1221;
        var t10 : Float = -m.e10 * e2233_2332 + m.e12 * e2033_2330 - m.e13 * e2032_2230;
        var t11 : Float = m.e00 * e2233_2332 - m.e02 * e2033_2330 + m.e03 * e2032_2230;
        var t12 : Float = -m.e00 * e1233_1332 + m.e02 * e1033_1330 - m.e03 * e1032_1230;
        var t13 : Float = m.e00 * e1223_1322 - m.e02 * e1023_1320 - m.e03 * e1220_2022;
        var t20 : Float = m.e10 * e2133_2331 - m.e11 * e2033_2330 + m.e13 * e2031_2130;
        var t21 : Float = -m.e00 * e2133_2331 + m.e01 * e2033_2330 - m.e03 * e2031_2130;
        var t22 : Float = m.e00 * e1133_1331 - m.e01 * e1033_1330 + m.e03 * e1031_1130;
        var t23 : Float = -m.e00 * e1123_1321 + m.e01 * e1023_1320 - m.e03 * e1021_1120;
        var t30 : Float = -m.e10 * e2132_2231 + m.e11 * e2032_2230 - m.e12 * e2031_2130;
        var t31 : Float = m.e00 * e2132_2231 - m.e01 * e2032_2230 + m.e02 * e2031_2130;
        var t32 : Float = -m.e00 * e1132_1231 + m.e01 * e1032_1230 - m.e02 * e1031_1130;
        var t33 : Float = m.e00 * e1122_1221 - m.e01 * e1022_1220 + m.e02 * e1021_1120;
        e00 = det * t00;
        e01 = det * t01;
        e02 = det * t02;
        e03 = det * t03;
        e10 = det * t10;
        e11 = det * t11;
        e12 = det * t12;
        e13 = det * t13;
        e20 = det * t20;
        e21 = det * t21;
        e22 = det * t22;
        e23 = det * t23;
        e30 = det * t30;
        e31 = det * t31;
        e32 = det * t32;
        e33 = det * t33;
        return this;
    }
    
    /**
		 * この行列を指定された値を元に作られるビュー変換行列に設定します。
		 * @param	eyeX 視点の x 座標
		 * @param	eyeY 視点の y 座標
		 * @param	eyeZ 視点の z 座標
		 * @param	atX 注視点の x 座標
		 * @param	atY 注視点の y 座標
		 * @param	atZ 注視点の z 座標
		 * @param	upX 上方向のベクトルの x 成分
		 * @param	upY 上方向のベクトルの y 成分
		 * @param	upZ 上方向のベクトルの z 成分
		 * @return このオブジェクト
		 */
    public function lookAt(
            eyeX : Float, eyeY : Float, eyeZ : Float,
            atX : Float, atY : Float, atZ : Float,
            upX : Float, upY : Float, upZ : Float) : Mat44{
        var zx : Float = eyeX - atX;
        var zy : Float = eyeY - atY;
        var zz : Float = eyeZ - atZ;
        var tmp : Float = 1 / Math.sqrt(zx * zx + zy * zy + zz * zz);
        zx *= tmp;
        zy *= tmp;
        zz *= tmp;
        var xx : Float = upY * zz - upZ * zy;
        var xy : Float = upZ * zx - upX * zz;
        var xz : Float = upX * zy - upY * zx;
        tmp = 1 / Math.sqrt(xx * xx + xy * xy + xz * xz);
        xx *= tmp;
        xy *= tmp;
        xz *= tmp;
        var yx : Float = zy * xz - zz * xy;
        var yy : Float = zz * xx - zx * xz;
        var yz : Float = zx * xy - zy * xx;
        e00 = xx;
        e01 = xy;
        e02 = xz;
        e03 = -(xx * eyeX + xy * eyeY + xz * eyeZ);
        e10 = yx;
        e11 = yy;
        e12 = yz;
        e13 = -(yx * eyeX + yy * eyeY + yz * eyeZ);
        e20 = zx;
        e21 = zy;
        e22 = zz;
        e23 = -(zx * eyeX + zy * eyeY + zz * eyeZ);
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
		 * この行列を指定された値を元に作られる透視投影変換行列に設定します。
		 * @param	fovY y 方向の視野角
		 * @param	aspect アスペクト比
		 * @param	near ニアクリップ面の距離
		 * @param	far ファークリップ面の距離
		 * @return このオブジェクト
		 */
    public function perspective(fovY : Float, aspect : Float, near : Float, far : Float) : Mat44{
        var h : Float = 1 / Math.tan(fovY * 0.5);
        var fnf : Float = far / (near - far);
        e00 = h / aspect;
        e01 = 0;
        e02 = 0;
        e03 = 0;
        e10 = 0;
        e11 = h;
        e12 = 0;
        e13 = 0;
        e20 = 0;
        e21 = 0;
        e22 = fnf;
        e23 = near * fnf;
        e30 = 0;
        e31 = 0;
        e32 = -1;
        e33 = 0;
        return this;
    }
    
    /**
		 * この行列を指定された値を元に作られる平行投影変換行列に設定します。
		 * @param	width 画面の幅
		 * @param	height 画面の高さ
		 * @param	near ニアクリップ面の距離
		 * @param	far ファークリップ面の距離
		 * @return このオブジェクト
		 */
    public function ortho(width : Float, height : Float, near : Float, far : Float) : Mat44{
        var nf : Float = 1 / (near - far);
        e00 = 2 / width;
        e01 = 0;
        e02 = 0;
        e03 = 0;
        e10 = 0;
        e11 = 2 / height;
        e12 = 0;
        e13 = 0;
        e20 = 0;
        e21 = 0;
        e22 = nf;
        e23 = near * nf;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 0;
        return this;
    }
    
    /**
		 * この行列の値を m からコピーします。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
    public function copy(m : Mat44) : Mat44{
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e03 = m.e03;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e13 = m.e13;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        e23 = m.e23;
        e30 = m.e30;
        e31 = m.e31;
        e32 = m.e32;
        e33 = m.e33;
        return this;
    }
    
    /**
		 * この行列の値を3行3列の要素を持つ行列 m からコピーします。
		 * 4行目および4列目の値は単位行列からコピーされます。
		 * @param	m 3行3列の要素を持つ行列
		 * @return このオブジェクト
		 */
    public function copyMat33(m : Mat33) : Mat44{
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e03 = 0;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e13 = 0;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        e23 = 0;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
		 * この Mat44 オブジェクトを複製します。
		 * @return 複製された Mat44 オブジェクト
		 */
    public function clone() : Mat44{
        return new Mat44(
		e00, e01, e02, e03, 
		e10, e11, e12, e13, 
		e20, e21, e22, e23, 
		e30, e31, e32, e33 
        );
    }
    
    /**
		 * この行列の文字列表現を返します。
		 * @return この行列を表す文字列
		 */
    public function toString() : String{
        var text : String = 
        "Mat44|" + Std.string(e00) + ", " + Std.string(e01) + ", " + Std.string(e02) + ", " + Std.string(e03) + "|\n" +
        "     |" + Std.string(e10) + ", " + Std.string(e11) + ", " + Std.string(e12) + ", " + Std.string(e13) + "|\n" +
        "     |" + Std.string(e20) + ", " + Std.string(e21) + ", " + Std.string(e22) + ", " + Std.string(e23) + "|\n" +
        "     |" + Std.string(e30) + ", " + Std.string(e31) + ", " + Std.string(e32) + ", " + Std.string(e33) + "|\n";
        return text;
    }
}

