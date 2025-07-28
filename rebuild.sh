#!/bin/bash

# Localization Workspace Rebuild Script
# Provides selective package building and cleaning capabilities
# Follows best practices to avoid unnecessary rebuilds

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Workspace directory
WORKSPACE_DIR="/home/dev/ORobotics/localization_ws"

# Package groups for common build sequences
CORE_PACKAGES="ublox_dgnss ublox_dgnss_node ublox_nav_sat_fix_hp_node wit_ros2_imu amr_sweeper_description gnss_imu_robot_localization"
RECORDER_PACKAGES="bag_recorder"
ALL_PACKAGES="$CORE_PACKAGES $RECORDER_PACKAGES"

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show help
show_help() {
    cat << EOF
Localization Workspace Rebuild Script

USAGE:
    ./rebuild.sh [OPTIONS] [PACKAGES...]

OPTIONS:
    --clean         Clean build artifacts before building
    --core          Build core localization packages only
    --recorder      Build bag recorder package only
    --all           Build all packages (default if no packages specified)
    --help, -h      Show this help message

PACKAGES:
    Specify one or more package names to build selectively

EXAMPLES:
    ./rebuild.sh                                    # Build all packages
    ./rebuild.sh bag_recorder                       # Build bag_recorder only
    ./rebuild.sh --clean gnss_imu_robot_localization # Clean and build specific package
    ./rebuild.sh --core                             # Build core packages only
    ./rebuild.sh package1 package2                  # Build multiple packages
    ./rebuild.sh --clean --all                      # Clean and rebuild everything

PACKAGE GROUPS:
    Core packages: $CORE_PACKAGES
    Recorder packages: $RECORDER_PACKAGES

SELECTIVE CLEANING:
    To clean specific packages without rebuilding:
    rm -rf build/package_name install/package_name

NOTES:
    - This script follows selective building best practices
    - Avoids 'rm -rf build/ install/ log/' to preserve working packages
    - Always sources the workspace after building
    - Use --clean only when necessary to save build time
EOF
}

# Function to clean specific packages
clean_packages() {
    local packages=("$@")
    
    if [ ${#packages[@]} -eq 0 ]; then
        print_warning "No packages specified for cleaning"
        return 0
    fi
    
    print_info "Cleaning build artifacts for packages: ${packages[*]}"
    
    for package in "${packages[@]}"; do
        if [ -d "build/$package" ] || [ -d "install/$package" ]; then
            print_info "Cleaning $package..."
            rm -rf "build/$package" "install/$package"
            # Clean related log files
            find log/ -name "*${package}*" -type f -delete 2>/dev/null || true
        else
            print_warning "No build artifacts found for $package"
        fi
    done
    
    print_success "Cleaning completed"
}

# Function to build packages
build_packages() {
    local packages=("$@")
    local build_cmd="colcon build"
    
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Building all packages..."
    else
        print_info "Building packages: ${packages[*]}"
        build_cmd="$build_cmd --packages-select ${packages[*]}"
    fi
    
    print_info "Executing: $build_cmd"
    
    if $build_cmd; then
        print_success "Build completed successfully"
        
        # Source the workspace
        if [ -f "install/setup.bash" ]; then
            print_info "Sourcing workspace..."
            source install/setup.bash
            print_success "Workspace sourced successfully"
        else
            print_warning "install/setup.bash not found, workspace not sourced"
        fi
    else
        print_error "Build failed"
        return 1
    fi
}

# Function to validate packages exist
validate_packages() {
    local packages=("$@")
    local invalid_packages=()
    
    for package in "${packages[@]}"; do
        if [ ! -d "src/$package" ]; then
            invalid_packages+=("$package")
        fi
    done
    
    if [ ${#invalid_packages[@]} -gt 0 ]; then
        print_error "The following packages do not exist in src/:"
        for pkg in "${invalid_packages[@]}"; do
            echo "  - $pkg"
        done
        print_info "Available packages:"
        ls -1 src/ | sed 's/^/  - /'
        return 1
    fi
    
    return 0
}

# Main function
main() {
    # Change to workspace directory
    if [ ! -d "$WORKSPACE_DIR" ]; then
        print_error "Workspace directory not found: $WORKSPACE_DIR"
        exit 1
    fi
    
    cd "$WORKSPACE_DIR"
    print_info "Working in: $(pwd)"
    
    # Parse command line arguments
    local clean_flag=false
    local packages=()
    local package_group=""
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                clean_flag=true
                shift
                ;;
            --core)
                package_group="core"
                shift
                ;;
            --recorder)
                package_group="recorder"
                shift
                ;;
            --all)
                package_group="all"
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            --*)
                print_error "Unknown option: $1"
                show_help
                exit 1
                ;;
            *)
                packages+=("$1")
                shift
                ;;
        esac
    done
    
    # Determine packages to build based on arguments
    local target_packages=()
    
    if [ -n "$package_group" ]; then
        case $package_group in
            core)
                read -ra target_packages <<< "$CORE_PACKAGES"
                ;;
            recorder)
                read -ra target_packages <<< "$RECORDER_PACKAGES"
                ;;
            all)
                target_packages=()  # Empty means build all
                ;;
        esac
    elif [ ${#packages[@]} -gt 0 ]; then
        target_packages=("${packages[@]}")
    else
        # Default: build all packages
        target_packages=()
    fi
    
    # Validate packages if specific packages were requested
    if [ ${#target_packages[@]} -gt 0 ]; then
        if ! validate_packages "${target_packages[@]}"; then
            exit 1
        fi
    fi
    
    # Clean if requested
    if [ "$clean_flag" = true ]; then
        if [ ${#target_packages[@]} -eq 0 ]; then
            print_warning "Clean flag with no specific packages - this would clean all packages"
            print_warning "Use 'rm -rf build/ install/ log/' if you really want to clean everything"
            print_info "Cleaning only packages that will be built..."
            # For safety, we'll clean all packages when building all
            read -ra clean_targets <<< "$ALL_PACKAGES"
            clean_packages "${clean_targets[@]}"
        else
            clean_packages "${target_packages[@]}"
        fi
    fi
    
    # Build packages
    build_packages "${target_packages[@]}"
    
    print_success "Rebuild script completed successfully"
    
    # Show next steps
    echo
    print_info "Next steps:"
    echo "  - Test your changes: ros2 launch gnss_imu_robot_localization bringup.launch.py"
    echo "  - Check topics: ros2 topic list"
    echo "  - Monitor logs: ros2 node list"
}

# Run main function with all arguments
main "$@"
