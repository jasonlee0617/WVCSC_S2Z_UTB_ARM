# ROSConDemo apple tree asset

- Source: https://github.com/o3de/ROSConDemo
- Source commit: `c788e1cddede404ba71d2c0319a0111cf46efa8f`
- Original files: `AppleTree.fbx`, `AppleTreeApples.fbx`, and their textures
- Copyright: Open 3D Engine ROSConDemo contributors
- License: Creative Commons Attribution-NonCommercial 4.0 International

Changes made for this project:

- selected the tree LOD2 and apple LOD1 meshes;
- converted FBX coordinates and units to Gazebo-compatible OBJ;
- duplicated leaf faces to retain the original double-sided rendering intent;
- replaced O3DE Atom materials with Gazebo/OGRE-compatible MTL materials;
- replaced complex mesh collision with a cylindrical trunk collision.

These derived assets are intended only for non-commercial research and competition demonstrations.
