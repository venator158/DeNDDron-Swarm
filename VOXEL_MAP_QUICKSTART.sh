#!/bin/bash
# Quick start commands for VoxelMap system

echo "VoxelMap Quick Start Commands"
echo "=============================="

AGENT_DIR="src/agent"

echo ""
echo "1. Install dependencies:"
echo "   cd $AGENT_DIR && pip install -r requirements.txt"

echo ""
echo "2. Generate test visualization (2D top-down view):"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --plot 2d --positions 20"

echo ""
echo "3. Generate interactive 3D visualization (HTML):"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --plot 3d_plotly --positions 20"
echo "   # Open index.html in the project root in your browser"

echo ""
echo "4. Generate all visualizations at once:"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --plot all --positions 20"

echo ""
echo "5. Export voxel map to CSV (for Excel/Pandas):"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --export-csv /tmp/voxels.csv"

echo ""
echo "6. Export voxel map to JSON:"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --export-json /tmp/voxels.json"

echo ""
echo "7. Record live agent voxel map snapshots:"
echo "   cd $AGENT_DIR && python3 record_voxel_map.py --agent drone_1 --interval 10 &"
echo "   # Later: python3 record_voxel_map.py --visualize --plot 3d_plotly"

echo ""
echo "8. Test all functionality:"
echo "   cd $AGENT_DIR && python3 test_voxel_map.py --positions 20 --plot all --export-csv /tmp/out.csv --export-json /tmp/out.json"

echo ""
echo "For detailed documentation, see: VOXEL_MAP_USAGE.md"
