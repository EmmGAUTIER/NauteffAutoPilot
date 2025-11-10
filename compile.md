# Compilation

## Outils de compilation
La compilation est réalisée avec cmake et arm-none-eabi-gcc.

## Initialisation de cmake
À la racine taper :
```
mkdir build
cd build
cmake ..
cd ..
```

## Commande de compilation
Depuis le répertoire principal taper
```
cmake --build build
```

Pour effacer les fichiers intermédiaires taper :
```
cmake --build build  -- target clean
```

