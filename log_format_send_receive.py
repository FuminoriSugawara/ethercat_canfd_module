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

# モジュールIDのエントリを抽出するためのパターン
pattern = re.compile(r"I \(\d+\) main: module_id: (\d+), Send count: (\d+), Receive count: (\d+)")

# データを保存する辞書
module_data = {}

# 各モジュールのデータを抽出
for line in cleaned_log_data:
    match = pattern.search(line)
    if match:
        module_id = int(match.group(1))
        send_count = int(match.group(2))
        receive_count = int(match.group(3))
        if module_id not in module_data:
            module_data[module_id] = []
        module_data[module_id].append([send_count, receive_count])

# 各モジュールIDごとに個別のCSVファイルに保存
for module_id, values in module_data.items():
    # ファイル名を作成
    file_name = f"module_{module_id}_counts.csv"
    file_path = os.path.join(output_directory, file_name)
    # DataFrameを作成
    df = pd.DataFrame(values, columns=['send_count', 'receive_count'])
    # CSVに保存
    df.to_csv(file_path, index=False, encoding='utf-8')

print("モジュールIDごとのデータの抽出とCSVへの保存が完了しました。")