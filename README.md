# BerryAPI
Berry code running on generation II 10 pin berries

#berry-lib
Code Composer Studio 6.1
Using the berry lib requires changing 2 settings in the project properties, then including the header file: berry.h
	Open the project properties and go to C/C++ Build>Settings
	Under "File Search Path," add the library berry.lib to "Include library or command file as input."  The file berry.lib is found under the project berry-lib/Debug. Add the file relative to the workspace.  
		*IMPORTANT* Add the library file above the existing libc.a, or it will not compile.  
	Under "Include Options," add the project folder berry-lib to the list of search path  directories.  
Now you can include berry.h and use the library.  See berry_template.c in berry-lib for example berry-specific code.

