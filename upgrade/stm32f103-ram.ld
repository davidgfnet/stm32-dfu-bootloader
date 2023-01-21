/*
 * This file is part of the libopenstm32 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Define memory regions. */
MEMORY
{
	/* Use up to 10KB for the program ("rom") and the rest for data. */
	rom (rx)  : ORIGIN = 0x20000000, LENGTH = 10240
	/* Reserve the last 8 bytes of RAM to save info across reboots */
	ram (rwx) : ORIGIN = 0x20002800, LENGTH = 10240 - 8
}

INCLUDE cortex-m-generic.ld

