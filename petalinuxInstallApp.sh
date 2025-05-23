#!/bin/bash

APP_NAME="${1:-}"

if [ -z "$APP_NAME" ]; then
    echo -e "\033[31mError: No app name\033[0m"
    exit 2
fi


RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m'

run_cmd() {
    cd /home/z/eclypse-os
    local step_name="$1"
    shift
    echo -e "${YELLOW}[步骤] 正在执行：${step_name}...${NC}"
    echo -e "命令：\033[35m$*\033[0m"
    
    if ! "$@"; then
        echo -e "${RED}错误：${step_name}失败 (退出码：$?)${NC}"
        return 1
    fi
    return 0
}

{
    run_cmd "完整系统构建" petalinux-build || exit $?
    
    
    run_cmd "应用程序构建安装" petalinux-build -c "$APP_NAME" -x do_install || exit $?
    
    
    run_cmd "根文件系统构建" petalinux-build -c rootfs || exit $?
    
    
    run_cmd "系统镜像打包" petalinux-build -x package || exit $?
    
    
    run_cmd "生成BOOT文件" petalinux-package --boot --fsbl --fpga --u-boot --force || exit $?
    
} || {
    echo -e "${RED}\n[Abort!] Check error information${NC}"
    exit 1
}


echo -e "${GREEN}\n[Success!]："
exit 0
