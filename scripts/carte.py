#!/usr/bin/env python3
import subprocess
import re
import sys
from collections import defaultdict

def parse_size_output(executable):
    """Analyse la sortie de la commande size"""
    result = subprocess.run(['arm-none-eabi-size', '-A', executable], 
                          capture_output=True, text=True)
    return result.stdout

def parse_nm_output(executable):
    """Analyse la sortie de nm pour obtenir les tailles des symboles"""
    result = subprocess.run(['arm-none-eabi-nm', '--size-sort', '--radix=d', '-S', executable],
                          capture_output=True, text=True)
    
    sections = defaultdict(list)
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
            
        # Format typique de nm avec -S: adresse taille t(type) nom
        parts = line.split()
        if len(parts) >= 4:
            try:
                # Les indices sont inversés par rapport à la version précédente
                addr = int(parts[0], 16)  # L'adresse est en premier, en hexa
                size = int(parts[1], 10)  # La taille est en second, en décimal
                sym_type = parts[2]
                name = parts[3]
                
                if sym_type in ['t', 'T']:  # Code
                    sections['text'].append((name, size, addr))
                elif sym_type in ['d', 'D']:  # Données initialisées
                    sections['data'].append((name, size, addr))
                elif sym_type in ['b', 'B']:  # BSS
                    sections['bss'].append((name, size, addr))
                elif sym_type in ['r', 'R']:  # Read-only data
                    sections['rodata'].append((name, size, addr))
            except (ValueError, IndexError):
                continue
    
    return sections

def format_size(size):
    """Formate une taille en bytes de façon lisible"""
    if size < 1024:
        return f"{size} B"
    elif size < 1024 * 1024:
        return f"{size/1024:.1f} KB"
    else:
        return f"{size/1024/1024:.1f} MB"

def generate_memory_map(executable):
    """Génère une carte mémoire détaillée"""
    print(f"\n=== Carte mémoire pour {executable} ===\n")
    
    # Afficher le résumé global
    print("Résumé global des sections:")
    print("-" * 80)
    print(parse_size_output(executable))
    print("-" * 80 + "\n")
    
    # Analyser les symboles par section
    sections = parse_nm_output(executable)
    
    for section_name, symbols in sections.items():
        if not symbols:
            continue
            
        total_size = sum(size for _, size, _ in symbols)
        print(f"\n{section_name.upper()} ({format_size(total_size)})")
        print("-" * 80)
        print(f"{'Adresse':<12} {'Taille':<12} {'Nom':<54}")
        print("-" * 80)
        
        # Trier par adresse croissante
        for name, size, addr in sorted(symbols, key=lambda x: x[2]):
            if size > 0:  # Ne montrer que les symboles avec une taille > 0
                print(f"{hex(addr):<12} {format_size(size):<12} {name[:54]:<54}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python carte.py <executable>")
        sys.exit(1)
    
    generate_memory_map(sys.argv[1])
