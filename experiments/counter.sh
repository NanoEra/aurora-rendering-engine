#!/usr/bin/env bash
# 代码行数统计器 - 增强版 (Bash 版)
# 尽量等效还原原 PowerShell 脚本功能与输出

set -uo pipefail

# 彩色输出映射（接近 PS 的颜色名）
declare -A COLOR_MAP=(
  ["Cyan"]="\e[36m"
  ["Green"]="\e[32m"
  ["Yellow"]="\e[33m"
  ["Magenta"]="\e[35m"
  ["Blue"]="\e[34m"
  ["Red"]="\e[31m"
  ["White"]="\e[97m"
  ["Gray"]="\e[90m"
  ["DarkYellow"]="\e[33m"
  ["Reset"]="\e[0m"
)

# 配置：类型 → 扩展列表、颜色
# 说明：扩展以 "*.ext" 形式书写；内部会派生出 ".ext" 用于匹配
declare -A TYPE_COLOR
declare -A TYPE_EXTS
TYPE_COLOR["C/C++"]="Cyan"
TYPE_EXTS["C/C++"]='*.c *.cpp *.cc *.cxx *.h *.hpp *.hh *.hxx'

TYPE_COLOR["C#"]="Green"
TYPE_EXTS["C#"]='*.cs'

TYPE_COLOR["Java"]="Yellow"
TYPE_EXTS["Java"]='*.java'

TYPE_COLOR["Python"]="Magenta"
TYPE_EXTS["Python"]='*.py *.pyw'

TYPE_COLOR["JavaScript"]="Blue"
TYPE_EXTS["JavaScript"]='*.js *.jsx *.ts *.tsx'

TYPE_COLOR["Web"]="Red"
TYPE_EXTS["Web"]='*.html *.htm *.css *.scss *.less *.sass'

TYPE_COLOR["Shell"]="White"
TYPE_EXTS["Shell"]='*.sh *.bash *.ps1 *.psm1 *.psd1'

TYPE_COLOR["配置/数据"]="Gray"
TYPE_EXTS["配置/数据"]='*.xml *.json *.yaml *.yml *.ini *.cfg *.config'

TYPE_COLOR["其他代码"]="DarkYellow"
TYPE_EXTS["其他代码"]='*.go *.rs *.rb *.php *.swift *.kt *.scala'

# 构建 扩展后缀(.ext) → 类型 的快速映射
declare -A SUFFIX_TYPE
for t in "${!TYPE_EXTS[@]}"; do
  for ext in ${TYPE_EXTS[$t]}; do
    # 将 "*.py" 转为 ".py"；统一小写匹配
    suf=".${ext#*.}"
    suf="$(printf '%s' "$suf" | tr '[:upper:]' '[:lower:]')"
    SUFFIX_TYPE["$suf"]="$t"
  done
done

# 标题
printf "%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%b       代码行数统计%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

# 当前目录
current_dir="$(pwd)"
printf "%b扫描目录: %s%b\n\n" "${COLOR_MAP[Gray]}" "$current_dir" "${COLOR_MAP[Reset]}"

# 统计变量
total_lines=0
total_files=0
declare -A type_files
declare -A type_lines
start_time_ns=$(date +%s%N)

# 初始化类型统计
for t in "${!TYPE_EXTS[@]}"; do
  type_files["$t"]=0
  type_lines["$t"]=0
done

# 收集全部文件（按各扩展累积）
# 说明：尽量贴近 PS：按类型/扩展多次递归收集再合并
# 使用 -iname 实现大小写不敏感匹配
mapfile -d '' ALL_FILES < <(
  {
    for t in "${!TYPE_EXTS[@]}"; do
      for ext in ${TYPE_EXTS[$t]}; do
        find "$current_dir" -type f -iname "${ext}" -print0 2>/dev/null
      done
    done
  } | awk -v RS='\0' '!seen[$0]++{printf "%s\0",$0}' # 去重，保持与 PS 相近的行为
)

total_to_process=${#ALL_FILES[@]}

if (( total_to_process == 0 )); then
  printf "%b未找到任何源代码文件！%b\n" "${COLOR_MAP[Red]}" "${COLOR_MAP[Reset]}"
  printf "%b请检查当前目录是否包含源代码文件。%b\n" "${COLOR_MAP[Gray]}" "${COLOR_MAP[Reset]}"
  exit 0
fi

printf "%b找到 %d 个文件需要处理...%b\n\n" "${COLOR_MAP[Gray]}" "$total_to_process" "${COLOR_MAP[Reset]}"

# 简易进度显示
processed=0

# 为格式对齐做准备
pad_right() { # $1=text $2=width
  local s="$1" w="$2" len=${#1}
  if (( len < w )); then printf "%s%*s" "$s" $((w-len)) ""; else printf "%s" "$s"; fi
}

# 统计循环
for file in "${ALL_FILES[@]}"; do
  ((processed++))
  # 百分比
  percent=$(( processed * 100 / total_to_process ))

  # 进度行（覆盖式）
  base="${file##*/}"
  printf "\r正在统计代码行数 | 处理文件: %s | 已处理: %d / %d | %3d%%" \
    "$(pad_right "$base" 30)" "$processed" "$total_to_process" "$percent"

  # 读取行数
  lines=0
  if ! lines=$(wc -l <"$file" 2>/dev/null); then
    # 模拟 PS 的警告输出（另起一行）
    printf "\n%b  警告: 无法读取文件 %s%b\n" "${COLOR_MAP[Red]}" "$base" "${COLOR_MAP[Reset]}"
    continue
  fi

  (( total_lines += lines ))
  (( total_files++ ))

  # 分类归属（根据扩展名后缀）
  fname_lower="$(printf '%s' "$file" | tr '[:upper:]' '[:lower:]')"
  # 取 ".ext"（含点），若没有点，则设为空
  suffix=""
  if [[ "$fname_lower" == *.* ]]; then
    suffix=".${fname_lower##*.}"
  fi

  t="${SUFFIX_TYPE[$suffix]:-}"
  if [[ -n "$t" ]]; then
    (( type_files["$t"] += 1 ))
    (( type_lines["$t"] += lines ))
  fi
done

# 完成进度条
printf "\n"

# 耗时
end_time_ns=$(date +%s%N)
duration_ns=$(( end_time_ns - start_time_ns ))
# 转为 mm:ss.mmm
total_ms=$(( duration_ns / 1000000 ))
mm=$(( (total_ms / 60000) % 60 ))
ss=$(( (total_ms / 1000) % 60 ))
ms=$(( total_ms % 1000 ))

# 详细统计标题
printf "\n%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%b            统计结果详情%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

# 按类型按行数降序输出
# 收集有文件的类型
types_sorted=()
while IFS= read -r line; do
  types_sorted+=("$line")
done < <(
  for t in "${!TYPE_EXTS[@]}"; do
    if (( type_files["$t"] > 0 )); then
      printf "%s\t%d\n" "$t" "${type_lines["$t"]}"
    fi
  done | sort -k2,2nr | awk -F'\t' '{print $1}'
)

for t in "${types_sorted[@]}"; do
  files=${type_files["$t"]}
  lines=${type_lines["$t"]}
  if (( files > 0 )); then
    percent=0
    if (( total_lines > 0 )); then
      # 保留两位小数
      percent=$(awk -v a="$lines" -v b="$total_lines" 'BEGIN{printf("%.2f", (b>0)?(a*100.0/b):0)}')
    fi
    color_name="${TYPE_COLOR[$t]}"
    color="${COLOR_MAP[$color_name]}"
    reset="${COLOR_MAP[Reset]}"
    printf "%b%-15s 文件: %4d  行数: %8d  (%6s%%)%b\n" "$color" "$t" "$files" "$lines" "$percent" "$reset"
  fi
done

# 汇总
printf "\n%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%b汇总统计%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

printf "%b总文件数:  %d%b\n" "${COLOR_MAP[Cyan]}" "$total_files" "${COLOR_MAP[Reset]}"
printf "%b总代码行:  %d%b\n" "${COLOR_MAP[Green]}" "$total_lines" "${COLOR_MAP[Reset]}"
printf "%b处理文件:  %d / %d%b\n" "${COLOR_MAP[Gray]}" "$processed" "$total_to_process" "${COLOR_MAP[Reset]}"
printf "%b统计耗时:  %02d:%02d.%03d%b\n" "${COLOR_MAP[Magenta]}" "$mm" "$ss" "$ms" "${COLOR_MAP[Reset]}"

# 文件大小统计
total_size_bytes=0
# 使用 stat 读取字节数（兼容 GNU/ BSD）
stat_size() {
  local f="$1"
  if stat --version >/dev/null 2>&1; then
    stat -c %s -- "$f"
  else
    stat -f %z -- "$f"
  fi
}
for f in "${ALL_FILES[@]}"; do
  if sz=$(stat_size "$f" 2>/dev/null); then
    (( total_size_bytes += sz ))
  fi
done
avg_size_bytes=0
if (( total_files > 0 )); then
  avg_size_bytes=$(( total_size_bytes / total_files ))
fi
to_mb=$(awk -v b="$total_size_bytes" 'BEGIN{printf("%.2f", b/1024/1024)}')
avg_kb=$(awk -v b="$avg_size_bytes"  'BEGIN{printf("%.2f", b/1024)}')

printf "%b总文件大小: %s MB%b\n" "${COLOR_MAP[Blue]}" "$to_mb" "${COLOR_MAP[Reset]}"
printf "%b平均文件大小: %s KB%b\n" "${COLOR_MAP[Blue]}" "$avg_kb" "${COLOR_MAP[Reset]}"

# 导出 CSV
printf "\n"
read -r -p "是否导出统计结果到CSV文件? (Y/N) " exportChoice
if [[ "$exportChoice" == "Y" || "$exportChoice" == "y" ]]; then
  timestamp="$(date +%Y%m%d_%H%M%S)"
  csv_path="${current_dir}/code_stats_${timestamp}.csv"
  {
    # 头部
    printf "文件类型,文件数量,代码行数,占比百分比\n"
    for t in "${types_sorted[@]}"; do
      files=${type_files["$t"]}
      lines=${type_lines["$t"]}
      if (( files > 0 )); then
        percent=$(awk -v a="$lines" -v b="$total_lines" 'BEGIN{printf("%.2f", (b>0)?(a*100.0/b):0)}')
        # 以 CSV 形式输出（简单场景，t 不含逗号）
        printf "%s,%d,%d,%s\n" "$t" "$files" "$lines" "$percent"
      fi
    done
    # 汇总行
    printf "总计,%d,%d,100\n" "$total_files" "$total_lines"
  } > "$csv_path"
  printf "%b统计结果已导出到: %s%b\n" "${COLOR_MAP[Green]}" "$csv_path" "${COLOR_MAP[Reset]}"
fi

# 任意键退出
printf "\n%b统计完成！按任意键退出...%b" "${COLOR_MAP[Gray]}" "${COLOR_MAP[Reset]}"
# shellcheck disable=SC2162
read -rsn1 _ 2>/dev/null || true
printf "\n"
