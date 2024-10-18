import re
import os
import pandas as pd

# 入力ファイルと出力ディレクトリの設定
input_file_path = 'log/output.log'  # ここにログファイルのパスを指定
output_directory = 'log/csv'  # 出力先のディレクトリを指定

# 出力ディレクトリが存在しない場合は作成
os.makedirs(output_directory, exist_ok=True)

# ログファイルを読み込む
with open(input_file_path, 'r', encoding='utf-8') as file:
    log_data = file.readlines()

# ログデータからエスケープシーケンスを削除
cleaned_log_data = [re.sub(r'\x1b\[\d+;?\d*?m', '', line) for line in log_data]

# "Stats"エントリを抽出するためのパターン
pattern_with_min_max_avg = re.compile(r"I \(\d+\) Stats: (.*?): min (\d+) us, max (\d+) us, avg (\d+) us")
pattern_overflow = re.compile(r"I \(\d+\) Stats: Overflow count: (\d+)")

# データを保存する辞書
stats_data_clean = {}

# "Stats"データを抽出
for line in cleaned_log_data:
    match_min_max_avg = pattern_with_min_max_avg.search(line)
    match_overflow = pattern_overflow.search(line)
    
    if match_min_max_avg:
        key = match_min_max_avg.group(1)
        min_val = int(match_min_max_avg.group(2))  # 数値部分のみ抽出
        max_val = int(match_min_max_avg.group(3))  # 数値部分のみ抽出
        avg_val = int(match_min_max_avg.group(4))  # 数値部分のみ抽出
        if key not in stats_data_clean:
            stats_data_clean[key] = {'type': 'min_max_avg', 'values': []}
        stats_data_clean[key]['values'].append([min_val, max_val, avg_val])
    
    elif match_overflow:
        key = 'Overflow_count'
        value = int(match_overflow.group(1))  # 数値部分のみ抽出
        if key not in stats_data_clean:
            stats_data_clean[key] = {'type': 'single_value', 'values': []}
        stats_data_clean[key]['values'].append([value])

# 各"Stats"データを個別のCSVファイルに保存
for key, data in stats_data_clean.items():
    file_name = f"{key.replace(' ', '_')}_cleaned.csv"
    file_path = os.path.join(output_directory, file_name)
    
    # DataFrameを作成
    if data['type'] == 'min_max_avg':
        df = pd.DataFrame(data['values'], columns=['min', 'max', 'avg'])
    else:  # 'single_value'
        df = pd.DataFrame(data['values'], columns=['value'])
    
    # CSVに保存
    df.to_csv(file_path, index=False, encoding='utf-8')

print("データの抽出とCSVへの保存が完了しました。")