# Installation

```bash
# Créer le dossier de build
mkdir build && cd build

# Configurer avec CMake
cmake ..

# Compiler
make
```

## Utilisation

Le programme propose 3 modes d'utilisation :

### 1. Mode Interactif

Traite une seule image avec des paramètres personnalisés.

**Syntaxe :**
```bash
./CoursOpenCv -i <image> [-g <ground_truth>]
```

**Exemple :**
```bash
cd build
./CoursOpenCv -i ../assets/tiger.pgm -g ../assets/gt/tiger_gt_binary.pgm
```

### 2. Mode Batch

Traite automatiquement toutes les images d'un dossier avec des paramètres fixes.

**Syntaxe :**
```bash
./CoursOpenCv -b <images_dir> <gt_dir>
```

**Exemple :**
```bash
cd build
./CoursOpenCv -b ../assets ../assets/gt
```

### 3. Mode Bruteforce

Optimisation automatique des paramètres pour chaque image.

**Syntaxe :**
```bash
./CoursOpenCv -bf <images_dir> <gt_dir>
```

**Exemple :**
```bash
cd build
./CoursOpenCv -bf ../assets ../assets/gt
```

