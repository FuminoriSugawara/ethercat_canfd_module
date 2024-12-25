#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import csv
import numpy as np

def main():
    # コマンドライン引数のチェック
    if len(sys.argv) < 2:
        print("Usage: python script.py <csv_file>")
        sys.exit(1)
    
    # CSVファイル名を取得
    csv_file = sys.argv[1]
    
    # CSVファイルからデータを読み込む
    # CSVファイルのフォーマット：1行ごとに「エンコーダ差分,トルク」の形式を想定
    x_values = []
    y_values = []
    
    with open(csv_file, 'r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            # 空行などをスキップ
            if not row:
                continue
            
            # 2列目まで揃っているかチェック
            if len(row) < 2:
                continue
            
            # CSV の値を float に変換して配列へ追加
            x_values.append(float(row[0]))
            y_values.append(float(row[1]))
    
    # numpy配列に変換
    x = np.array(x_values)
    y = np.array(y_values)
    
    # ----- 原点を通る三次近似 -----
    # T(x) = a1*x + a2*x^2 + a3*x^3  （a0=0 を固定）
    
    # デザイン行列（列順序は [x, x^2, x^3] ）
    M = np.column_stack([x, x**2, x**3])
    
    # 最小二乗解を求める
    # np.linalg.lstsq(M, y, rcond=None) は
    # 係数ベクトル [a1, a2, a3], 残差, ランク, 特異値 を返す
    coeffs, residuals, rank, s = np.linalg.lstsq(M, y, rcond=None)
    
    # coeffs に [a1, a2, a3] が格納される
    a1, a2, a3 = coeffs
    
    # 近似関数を定義
    def T(x_val):
        return a1*x_val + a2*(x_val**2) + a3*(x_val**3)
    
    # 結果表示
    print("--------------------------------------------------")
    print("【フィッティング結果】（原点通過を強制した三次多項式）")
    print("T(x) = a1*x + a2*x^2 + a3*x^3")
    print(f"a1 = {a1}")
    print(f"a2 = {a2}")
    print(f"a3 = {a3}")
    print("--------------------------------------------------")
    
    # 近似の残差二乗和などを確認したい場合（オプション）
    # ※ np.linalg.lstsq の residuals には (y - M@coeffs) の二乗和が入る（サンプル数が係数数より大きい場合）
    if len(residuals) > 0:
        print(f"Residual sum of squares: {residuals[0]}")
    
    # テスト: エンコーダ差分=50 のとき
    test_encoder_diff = 50
    predicted_torque = T(test_encoder_diff)
    print(f"\nエンコーダ差分 {test_encoder_diff:.2f} に対する推定トルク: {predicted_torque:.3f} Nm")
    print("--------------------------------------------------")

if __name__ == "__main__":
    main()