from juliacall import Main as jl

jl.Pkg.status()

jl.seval("using AbstractGPs")

jl.Pkg.status()