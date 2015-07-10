//=============================================================================================
// 6-Axis Robot Arm Motion Engine
// Copyright 2015 by Glen Akins.
// All rights reserved.
// 
// Set editor width to 96 and tab stop to 4.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//=============================================================================================

#define MOTION_BUSY			0xc0000000
#define MOTION_PRGM_REQ		0xc0000004
#define MOTION_ALARM		0xc0000008

#define MX_PRGM_DIR			0xc0000020
#define MX_PRGM_ACC			0xc0000024
#define MX_PRGM_ASAMPS		0xc0000028
#define MX_PRGM_CSAMPS		0xc000002c
#define MX_LIMIT_LO			0xc0000030
#define MX_LIMIT_HI			0xc0000034
#define MX_POSITION			0xc0000038

#define MY_PRGM_DIR			0xc0000040
#define MY_PRGM_ACC			0xc0000044
#define MY_PRGM_ASAMPS		0xc0000048
#define MY_PRGM_CSAMPS		0xc000004c
#define MY_LIMIT_LO			0xc0000050
#define MY_LIMIT_HI			0xc0000054
#define MY_POSITION			0xc0000058

#define MZ_PRGM_DIR			0xc0000060
#define MZ_PRGM_ACC			0xc0000064
#define MZ_PRGM_ASAMPS		0xc0000068
#define MZ_PRGM_CSAMPS		0xc000006c
#define MZ_LIMIT_LO			0xc0000070
#define MZ_LIMIT_HI			0xc0000074
#define MZ_POSITION			0xc0000078

#define MA_PRGM_DIR			0xc0000080
#define MA_PRGM_ACC			0xc0000084
#define MA_PRGM_ASAMPS		0xc0000088
#define MA_PRGM_CSAMPS		0xc000008c
#define MA_LIMIT_LO			0xc0000090
#define MA_LIMIT_HI			0xc0000094
#define MA_POSITION			0xc0000098

#define MB_PRGM_DIR			0xc00000a0
#define MB_PRGM_ACC			0xc00000a4
#define MB_PRGM_ASAMPS		0xc00000a8
#define MB_PRGM_CSAMPS		0xc00000ac
#define MB_LIMIT_LO			0xc00000b0
#define MB_LIMIT_HI			0xc00000b4
#define MB_POSITION			0xc00000b8

#define MC_PRGM_DIR			0xc00000c0
#define MC_PRGM_ACC			0xc00000c4
#define MC_PRGM_ASAMPS		0xc00000c8
#define MC_PRGM_CSAMPS		0xc00000cc
#define MC_LIMIT_LO			0xc00000d0
#define MC_LIMIT_HI			0xc00000d4
#define MC_POSITION			0xc00000d8
