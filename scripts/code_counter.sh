#!/usr/bin/env bash

set -uo pipefail

# Colors map
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

# Configuration: Type → Extension list, Color
declare -A TYPE_COLOR
declare -A TYPE_EXTS
TYPE_COLOR["C/C++"]="Cyan"
TYPE_EXTS["C/C++"]='c cpp cc cxx h hpp hh hxx'

TYPE_COLOR["C#"]="Green"
TYPE_EXTS["C#"]='cs'

TYPE_COLOR["Java"]="Yellow"
TYPE_EXTS["Java"]='java'

TYPE_COLOR["Python"]="Magenta"
TYPE_EXTS["Python"]='py pyw'

TYPE_COLOR["JavaScript"]="Blue"
TYPE_EXTS["JavaScript"]='js jsx ts tsx'

TYPE_COLOR["Web"]="Red"
TYPE_EXTS["Web"]='html htm css scss less sass'

TYPE_COLOR["Shell"]="White"
TYPE_EXTS["Shell"]='sh bash ps1 psm1 psd1'

TYPE_COLOR["Config/Data"]="Gray"
TYPE_EXTS["Config/Data"]='xml json yaml yml ini cfg config'

TYPE_COLOR["Other Code"]="DarkYellow"
TYPE_EXTS["Other Code"]='go rs rb php swift kt scala'

# Show usage
show_usage() {
	printf "Usage: %s [OPTIONS] [DIRECTORY]\n\n" "$(basename "$0")"
	printf "Options:\n"
	printf "  -h, --help     Show this help message\n"
	printf "  -d, --dir DIR  Specify directory to scan\n"
	printf "\nExamples:\n"
	printf "  %s                    # Scan current directory\n" "$(basename "$0")"
	printf "  %s /path/to/project   # Scan specified directory\n" "$(basename "$0")"
	printf "  %s -d /path/to/project\n" "$(basename "$0")"
}

# Parse command line arguments
scan_dir=""
while [[ $# -gt 0 ]]; do
	case "$1" in
		-h|--help)
			show_usage
			exit 0
			;;
		-d|--dir)
			if [[ -n "${2:-}" ]]; then
				scan_dir="$2"
				shift 2
			else
				printf "Error: --dir requires a directory argument\n" >&2
				exit 1
			fi
			;;
		-*)
			printf "Error: Unknown option: %s\n" "$1" >&2
			show_usage
			exit 1
			;;
		*)
			if [[ -z "$scan_dir" ]]; then
				scan_dir="$1"
			else
				printf "Error: Multiple directories specified\n" >&2
				exit 1
			fi
			shift
			;;
	esac
done

# Default to current directory if not specified
if [[ -z "$scan_dir" ]]; then
	scan_dir="$(pwd)"
fi

# Validate directory
if [[ ! -d "$scan_dir" ]]; then
	printf "Error: Directory does not exist: %s\n" "$scan_dir" >&2
	exit 1
fi

# Convert to absolute path using readlink or realpath
if command -v realpath >/dev/null 2>&1; then
	target_dir="$(realpath "$scan_dir")"
elif command -v readlink >/dev/null 2>&1 && readlink -f / >/dev/null 2>&1; then
	target_dir="$(readlink -f "$scan_dir")"
else
	# Fallback: use cd + pwd
	target_dir="$(cd "$scan_dir" && pwd)"
fi

if [[ -z "$target_dir" || ! -d "$target_dir" ]]; then
	printf "Error: Cannot resolve directory: %s\n" "$scan_dir" >&2
	exit 1
fi

# Build suffix → type mapping (lowercase, without dot)
declare -A SUFFIX_TYPE
for t in "${!TYPE_EXTS[@]}"; do
	for ext in ${TYPE_EXTS[$t]}; do
		ext_lower="$(printf '%s' "$ext" | tr '[:upper:]' '[:lower:]')"
		SUFFIX_TYPE["$ext_lower"]="$t"
	done
done

# Build regex pattern for find
# Create a single regex pattern matching all extensions
all_extensions=()
for t in "${!TYPE_EXTS[@]}"; do
	for ext in ${TYPE_EXTS[$t]}; do
		all_extensions+=("$ext")
	done
done

# Title
printf "%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%b       Code Line Counter%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

printf "%bScanning directory: %s%b\n\n" "${COLOR_MAP[Gray]}" "$target_dir" "${COLOR_MAP[Reset]}"

# Statistics variables
total_lines=0
total_files=0
declare -A type_files
declare -A type_lines
declare -A ext_files
declare -A ext_lines
declare -a FILE_INFO_LIST
start_time_ns=$(date +%s%N)

# Initialize type statistics
for t in "${!TYPE_EXTS[@]}"; do
	type_files["$t"]=0
	type_lines["$t"]=0
done

printf "%bCollecting files...%b\n" "${COLOR_MAP[Gray]}" "${COLOR_MAP[Reset]}"

# Collect ALL regular files first, then filter by extension
# This avoids any issues with find's -iname pattern matching
declare -A SEEN_FILES
ALL_FILES=()

# Use find to get all files, then filter in bash
# This is more reliable than multiple find calls with -iname
while IFS= read -r -d '' file; do
	# Skip empty
	[[ -z "$file" ]] && continue
	
	# Get filename and extension
	filename="${file##*/}"
	
	# Skip files without extension
	[[ "$filename" != *.* ]] && continue
	
	# Get extension (lowercase, without dot)
	ext="${filename##*.}"
	ext_lower="$(printf '%s' "$ext" | tr '[:upper:]' '[:lower:]')"
	
	# Check if this extension is in our list
	if [[ -n "${SUFFIX_TYPE[$ext_lower]+isset}" ]]; then
		# Deduplicate
		if [[ -z "${SEEN_FILES["$file"]+isset}" ]]; then
			SEEN_FILES["$file"]=1
			ALL_FILES+=("$file")
		fi
	fi
done < <(find "$target_dir" -type f -print0 2>/dev/null)

total_to_process=${#ALL_FILES[@]}

if (( total_to_process == 0 )); then
	printf "%bNo source code files found!%b\n" "${COLOR_MAP[Red]}" "${COLOR_MAP[Reset]}"
	printf "%bPlease check if the directory contains source code files.%b\n" "${COLOR_MAP[Gray]}" "${COLOR_MAP[Reset]}"
	exit 0
fi

printf "%bFound %d files to process...%b\n\n" "${COLOR_MAP[Gray]}" "$total_to_process" "${COLOR_MAP[Reset]}"

processed=0

pad_right() {
	local s="$1" w="$2" len=${#1}
	if (( len < w )); then
		printf "%s%*s" "$s" $((w-len)) ""
	else
		printf "%s" "$s"
	fi
}

count_non_empty_lines() {
	local file="$1"
	local count
	count=$(grep -c '[^[:space:]]' "$file" 2>/dev/null) || count=0
	printf '%d' "$count"
}

# Statistics loop
for file in "${ALL_FILES[@]}"; do
	((processed++))
	percent=$(( processed * 100 / total_to_process ))

	base="${file##*/}"
	printf "\rCounting code lines | Processing: %s | Progress: %d / %d | %3d%%" \
		"$(pad_right "$base" 30)" "$processed" "$total_to_process" "$percent"

	if [[ ! -r "$file" ]]; then
		printf "\n%b  Warning: Cannot read file %s%b\n" "${COLOR_MAP[Red]}" "$base" "${COLOR_MAP[Reset]}"
		continue
	fi

	lines=$(count_non_empty_lines "$file")

	(( total_lines += lines ))
	(( total_files++ ))

	# Get extension (lowercase, without dot)
	ext="${base##*.}"
	ext_lower="$(printf '%s' "$ext" | tr '[:upper:]' '[:lower:]')"
	suffix=".${ext_lower}"

	t="${SUFFIX_TYPE[$ext_lower]:-}"
	if [[ -n "$t" ]]; then
		(( type_files["$t"] += 1 ))
		(( type_lines["$t"] += lines ))

		ext_key="${t}|${suffix}"
		(( ext_files["$ext_key"] = ${ext_files["$ext_key"]:-0} + 1 ))
		(( ext_lines["$ext_key"] = ${ext_lines["$ext_key"]:-0} + lines ))

		file_dir="$(dirname "$file")"
		file_name="$(basename "$file")"
		FILE_INFO_LIST+=("${t}|${suffix}|${file_dir}|${file_name}|${lines}")
	fi
done

printf "\n"

# Duration
end_time_ns=$(date +%s%N)
duration_ns=$(( end_time_ns - start_time_ns ))
total_ms=$(( duration_ns / 1000000 ))
mm=$(( (total_ms / 60000) % 60 ))
ss=$(( (total_ms / 1000) % 60 ))
ms=$(( total_ms % 1000 ))

# Detailed statistics
printf "\n%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%b         Statistics Details%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

types_sorted=()
while IFS= read -r line; do
	[[ -n "$line" ]] && types_sorted+=("$line")
done < <(
	for t in "${!TYPE_EXTS[@]}"; do
		if (( ${type_files["$t"]:-0} > 0 )); then
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
			percent=$(awk -v a="$lines" -v b="$total_lines" 'BEGIN{printf("%.2f", (b>0)?(a*100.0/b):0)}')
		fi
		color_name="${TYPE_COLOR[$t]}"
		color="${COLOR_MAP[$color_name]}"
		reset="${COLOR_MAP[Reset]}"
		printf "%b%-15s Files: %4d  Lines: %8d  (%6s%%)%b\n" "$color" "$t" "$files" "$lines" "$percent" "$reset"
	fi
done

# Summary
printf "\n%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"
printf "%bSummary Statistics%b\n" "${COLOR_MAP[Yellow]}" "${COLOR_MAP[Reset]}"
printf "%b========================================%b\n" "${COLOR_MAP[White]}" "${COLOR_MAP[Reset]}"

printf "%bTotal Files:  %d%b\n" "${COLOR_MAP[Cyan]}" "$total_files" "${COLOR_MAP[Reset]}"
printf "%bTotal Lines:  %d%b\n" "${COLOR_MAP[Green]}" "$total_lines" "${COLOR_MAP[Reset]}"
printf "%bProcessed:    %d / %d%b\n" "${COLOR_MAP[Gray]}" "$processed" "$total_to_process" "${COLOR_MAP[Reset]}"
printf "%bDuration:     %02d:%02d.%03d%b\n" "${COLOR_MAP[Magenta]}" "$mm" "$ss" "$ms" "${COLOR_MAP[Reset]}"

# File size statistics
total_size_bytes=0
stat_size() {
	local f="$1"
	if stat --version >/dev/null 2>&1; then
		stat -c %s -- "$f" 2>/dev/null
	else
		stat -f %z -- "$f" 2>/dev/null
	fi
}
for f in "${ALL_FILES[@]}"; do
	if sz=$(stat_size "$f"); then
		(( total_size_bytes += sz ))
	fi
done
avg_size_bytes=0
if (( total_files > 0 )); then
	avg_size_bytes=$(( total_size_bytes / total_files ))
fi
to_mb=$(awk -v b="$total_size_bytes" 'BEGIN{printf("%.2f", b/1024/1024)}')
avg_kb=$(awk -v b="$avg_size_bytes"  'BEGIN{printf("%.2f", b/1024)}')

printf "%bTotal File Size: %s MB%b\n" "${COLOR_MAP[Blue]}" "$to_mb" "${COLOR_MAP[Reset]}"
printf "%bAverage File Size: %s KB%b\n" "${COLOR_MAP[Blue]}" "$avg_kb" "${COLOR_MAP[Reset]}"

# Export CSV
printf "\n"
read -r -p "Export statistics to CSV file? (Y/N) " exportChoice
if [[ "$exportChoice" == "Y" || "$exportChoice" == "y" ]]; then
	timestamp="$(date +%Y%m%d_%H%M%S)"
	csv_path="${target_dir}/code_stats_${timestamp}.csv"

	{
		# CSV Header for file details
		printf "Directory,File Name,Language,Extension,Lines\n"

		# Sort by directory -> language -> extension -> filename
		printf '%s\n' "${FILE_INFO_LIST[@]}" | sort -t'|' -k3,3 -k1,1 -k2,2 -k4,4 | while IFS='|' read -r ftype fsuffix fdir fname flines; do
			[[ -z "$ftype" ]] && continue

			# Escape fields for CSV
			escaped_dir="${fdir//\"/\"\"}"
			escaped_fname="${fname//\"/\"\"}"
			escaped_type="${ftype//\"/\"\"}"
			escaped_suffix="${fsuffix//\"/\"\"}"

			printf '"%s","%s","%s","%s",%d\n' \
				"$escaped_dir" "$escaped_fname" "$escaped_type" "$escaped_suffix" "$flines"
		done

		printf "\n"
		printf -- "--- Extension Summary ---\n"
		printf "Language,Extension,File Count,Line Count,Percentage\n"
		for t in "${types_sorted[@]}"; do
			for key in "${!ext_lines[@]}"; do
				if [[ "$key" == "${t}|"* ]]; then
					suffix="${key#*|}"
					ext_file_count=${ext_files["$key"]:-0}
					ext_line_count=${ext_lines["$key"]:-0}
					if (( ext_file_count > 0 )); then
						ext_percent=$(awk -v a="$ext_line_count" -v b="$total_lines" 'BEGIN{printf("%.2f", (b>0)?(a*100.0/b):0)}')
						escaped_t="${t//\"/\"\"}"
						escaped_s="${suffix//\"/\"\"}"
						printf '"%s","%s",%d,%d,%s\n' "$escaped_t" "$escaped_s" "$ext_file_count" "$ext_line_count" "$ext_percent"
					fi
				fi
			done
		done

		printf "\n"
		printf -- "--- Language Summary ---\n"
		printf "Language,File Count,Line Count,Percentage\n"
		for t in "${types_sorted[@]}"; do
			files=${type_files["$t"]}
			lines=${type_lines["$t"]}
			if (( files > 0 )); then
				percent=$(awk -v a="$lines" -v b="$total_lines" 'BEGIN{printf("%.2f", (b>0)?(a*100.0/b):0)}')
				escaped_t="${t//\"/\"\"}"
				printf '"%s",%d,%d,%s\n' "$escaped_t" "$files" "$lines" "$percent"
			fi
		done

		printf "\n"
		printf -- "--- Total ---\n"
		printf "Total Files,Total Lines\n"
		printf "%d,%d\n" "$total_files" "$total_lines"

	} > "$csv_path"

	printf "%bStatistics exported to: %s%b\n" "${COLOR_MAP[Green]}" "$csv_path" "${COLOR_MAP[Reset]}"
fi

printf "\n%bStatistics complete! Press any key to exit...%b" "${COLOR_MAP[Gray]}" "${COLOR_MAP[Reset]}"
read -rsn1 _ 2>/dev/null || true
printf "\n"
