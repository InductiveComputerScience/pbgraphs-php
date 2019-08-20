<?php

require_once("DirectedGraphs.php");


$directedGraph = MakeGraphForDijkstrasAlgorithm();

$d = new stdClass();
$p = new stdClass();
$ds = new stdClass();
DijkstrasAlgorithm($directedGraph, 0.0, $d, $ds, $p);

print_r($d);