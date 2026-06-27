"""
Video Optimization Script for Space Module Dual-Arm Assembly
===========================================================
Optimizes the demo video with:
1. GIF preview generation
2. Bottom metrics overlay
3. Reverse opening (show final result first)
4. HUD sensor overlay
"""

import subprocess
import json
from pathlib import Path


def optimize_video():
    """Main optimization function."""
    print("="*60)
    print("VIDEO OPTIMIZATION")
    print("="*60)
    
    # Step 1: Generate GIF preview
    print("\n[1/4] Generating GIF preview...")
    generate_gif_preview()
    
    # Step 2: Add bottom metrics overlay
    print("\n[2/4] Adding bottom metrics overlay...")
    add_bottom_metrics()
    
    # Step 3: Create reverse opening
    print("\n[3/4] Creating reverse opening...")
    create_reverse_opening()
    
    # Step 4: Add HUD sensor overlay
    print("\n[4/4] Adding HUD sensor overlay...")
    add_hud_overlay()
    
    print("\n" + "="*60)
    print("OPTIMIZATION COMPLETE")
    print("="*60)


def generate_gif_preview():
    """Generate GIF preview from first 10 seconds."""
    input_file = "demo.mp4"
    output_file = "demo_preview.gif"
    
    # Extract first 10 seconds and convert to GIF
    cmd = [
        "ffmpeg", "-y",
        "-i", input_file,
        "-t", "10",  # First 10 seconds
        "-vf", "fps=10,scale=640:-1:flags=lanczos",  # 10fps, 640px wide
        "-loop", "0",  # Infinite loop
        output_file
    ]
    
    try:
        subprocess.run(cmd, capture_output=True, check=True)
        print(f"  ✓ Generated: {output_file}")
        
        # Check file size
        size = Path(output_file).stat().st_size
        print(f"    Size: {size / 1024:.1f} KB")
    except subprocess.CalledProcessError as e:
        print(f"  ✗ Failed: {e}")


def add_bottom_metrics():
    """Add bottom metrics overlay to video."""
    input_file = "demo.mp4"
    output_file = "demo_with_metrics.mp4"
    
    # Metrics text
    metrics_text = (
        "Success Rate: 100% (128/128) | "
        "Wilson CI: [97.1%, 100%] | "
        "Force RMSE: 5.23N | "
        "Decision Freq: 8.0 Hz | "
        "Physics Audit: 7/8 Passed"
    )
    
    # Add text overlay at bottom
    cmd = [
        "ffmpeg", "-y",
        "-i", input_file,
        "-vf", (
            f"drawtext=text='{metrics_text}':"
            "fontcolor=white:"
            "fontsize=18:"
            "box=1:"
            "boxcolor=black@0.7:"
            "boxborderw=5:"
            "x=(w-text_w)/2:"
            "y=h-th-10"
        ),
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "23",
        output_file
    ]
    
    try:
        subprocess.run(cmd, capture_output=True, check=True)
        print(f"  ✓ Generated: {output_file}")
        
        # Check file size
        size = Path(output_file).stat().st_size
        print(f"    Size: {size / 1024 / 1024:.1f} MB")
    except subprocess.CalledProcessError as e:
        print(f"  ✗ Failed: {e}")


def create_reverse_opening():
    """Create video with reverse opening (final result first)."""
    input_file = "demo.mp4"
    output_file = "demo_with_opening.mp4"
    
    # Extract last 5 seconds (final stack result)
    cmd_extract = [
        "ffmpeg", "-y",
        "-i", input_file,
        "-ss", "26",  # Start from 26 seconds
        "-t", "5",  # 5 seconds
        "-c", "copy",
        "temp_ending.mp4"
    ]
    
    # Extract main content (0-26 seconds)
    cmd_main = [
        "ffmpeg", "-y",
        "-i", input_file,
        "-t", "26",
        "-c", "copy",
        "temp_main.mp4"
    ]
    
    # Add title overlay to ending
    cmd_title = [
        "ffmpeg", "-y",
        "-i", "temp_ending.mp4",
        "-vf", (
            "drawtext=text='Final Result: 3-Module Stack Complete':"
            "fontcolor=yellow:"
            "fontsize=24:"
            "box=1:"
            "boxcolor=black@0.8:"
            "boxborderw=10:"
            "x=(w-text_w)/2:"
            "y=50"
        ),
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "23",
        "temp_ending_titled.mp4"
    ]
    
    # Concatenate: ending first, then main
    cmd_concat = [
        "ffmpeg", "-y",
        "-i", "temp_ending_titled.mp4",
        "-i", "temp_main.mp4",
        "-filter_complex", "[0:v][1:v]concat=n=2:v=1:a=0[outv]",
        "-map", "[outv]",
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "23",
        output_file
    ]
    
    try:
        # Extract ending
        subprocess.run(cmd_extract, capture_output=True, check=True)
        print("  ✓ Extracted ending clip")
        
        # Extract main
        subprocess.run(cmd_main, capture_output=True, check=True)
        print("  ✓ Extracted main content")
        
        # Add title to ending
        subprocess.run(cmd_title, capture_output=True, check=True)
        print("  ✓ Added title to ending")
        
        # Concatenate
        subprocess.run(cmd_concat, capture_output=True, check=True)
        print(f"  ✓ Generated: {output_file}")
        
        # Check file size
        size = Path(output_file).stat().st_size
        print(f"    Size: {size / 1024 / 1024:.1f} MB")
        
        # Cleanup temp files
        for f in ["temp_ending.mp4", "temp_main.mp4", "temp_ending_titled.mp4"]:
            Path(f).unlink(missing_ok=True)
        
    except subprocess.CalledProcessError as e:
        print(f"  ✗ Failed: {e}")


def add_hud_overlay():
    """Add HUD sensor overlay to video."""
    input_file = "demo_with_opening.mp4"
    output_file = "demo_final.mp4"
    
    # HUD overlay with sensor data
    hud_text = (
        "DOF: 14 | "
        "Force: 5.23N | "
        "Freq: 8.0Hz | "
        "Steps: 22/22 | "
        "Audit: 7/8"
    )
    
    # Add HUD overlay at top-right
    cmd = [
        "ffmpeg", "-y",
        "-i", input_file,
        "-vf", (
            f"drawtext=text='{hud_text}':"
            "fontcolor=cyan:"
            "fontsize=16:"
            "box=1:"
            "boxcolor=black@0.6:"
            "boxborderw=5:"
            "x=w-tw-10:"
            "y=10"
        ),
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "23",
        output_file
    ]
    
    try:
        subprocess.run(cmd, capture_output=True, check=True)
        print(f"  ✓ Generated: {output_file}")
        
        # Check file size
        size = Path(output_file).stat().st_size
        print(f"    Size: {size / 1024 / 1024:.1f} MB")
    except subprocess.CalledProcessError as e:
        print(f"  ✗ Failed: {e}")


if __name__ == "__main__":
    optimize_video()
