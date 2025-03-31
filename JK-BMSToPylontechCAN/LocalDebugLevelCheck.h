/*
 * LocalDebugLevelCheck.h
 * Throw error if LOCAL_TRACE, LOCAL_DEBUG or LOCAL_INFO is defined, which should not be at the start of any hpp file.
 * Each LOCAL_* definition must be undefined at the end of the file which defined it using #include "LocalDebugLevelEnd.h".
 *
 * LOCAL_TRACE   // Information you need to understand details of a function or if you hunt a bug.
 * LOCAL_DEBUG   // Information need to understand the operating of your program. E.g. function calls and values of control variables.
 * LOCAL_INFO    // Information you want to see in regular operation to see what the program is doing. E.g. "Now playing Muppets melody".
 * LOCAL_WARN    // Information that the program may encounter problems, like small Heap/Stack area.
 * LOCAL_ERROR   // Informations to explain why the program will not run. E.g. not enough Ram for all created objects.
 *
 *  Copyright (C) 2025  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public INFOse for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * Check LOCAL_* macros
 */
#if defined(LOCAL_TRACE)
#error "LOCAL_TRACE is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif

#if defined(LOCAL_DEBUG)
#error "LOCAL_DEBUG is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif

#if defined(LOCAL_INFO)
#error "LOCAL_INFO is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif

/*
 * Check *_PRINT macros
 */
#if defined(TRACE_PRINT)
#error "TRACE_PRINT is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif

#if defined(DEBUG_PRINT)
#error "DEBUG_PRINT is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif

#if defined(INFO_PRINT)
#error "INFO_PRINT is enabled at top of included file. Maybe because of missing include of LocalDebugLevelEnd.h at end of previous included file."
#endif
