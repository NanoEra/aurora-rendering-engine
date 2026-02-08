#!/bin/bash

# query.sh - 遍历指定文件夹中的 .h 文件并生成 all_headers.md

# 检查是否提供了目录参数
if [ $# -ne 1 ]; then
    echo "用法: $0 <目标文件夹路径>"
    exit 1
fi

TARGET_DIR="$1"

# 检查提供的路径是否为一个存在的目录
if [ ! -d "$TARGET_DIR" ]; then
    echo "错误: 目录 '$TARGET_DIR' 不存在。"
    exit 1
fi

# 输出文件
OUTPUT_FILE="all_headers.md"

# 清空或创建输出文件
> "$OUTPUT_FILE"

echo "正在扫描目录: $TARGET_DIR"
# 使用 find 命令查找所有 .h 文件
H_FILES=$(find "$TARGET_DIR" -type f -name "*.h")

# 检查是否找到了 .h 文件
if [ -z "$H_FILES" ]; then
    echo "在目录 '$TARGET_DIR' 及其子目录中未找到任何 .h 文件。"
    exit 0
fi

# 遍历找到的每个 .h 文件
for header_file in $H_FILES; do
    # 获取相对于脚本执行位置的相对路径
    RELATIVE_PATH=$(realpath --relative-to=. "$header_file")
    
    # 写入分隔符和文件名
    {
        echo "### 文件：$RELATIVE_PATH"
        echo ""
        echo '```cpp'
        cat "$header_file"
        echo '```'
        echo "" # 添加一个空行，使文件之间有分隔
    } >> "$OUTPUT_FILE"
    
    echo "已处理: $RELATIVE_PATH"
done

echo ""
echo "处理完成！所有头文件内容已合并到 $OUTPUT_FILE 中。"
