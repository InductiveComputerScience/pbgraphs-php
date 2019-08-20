<?php

function unichr($unicode){
    return mb_convert_encoding("&#{$unicode};", 'UTF-8', 'HTML-ENTITIES');
}
function uniord($s) {
    return unpack('V', iconv('UTF-8', 'UCS-4LE', $s))[1];
}

function DepthFirstSearch($g, $start, $list){

  $visited = CreateBooleanArray(count($g->nodes), false);
  $ll = CreateLinkedListNumbers();

  DepthFirstSearchRecursive($g, $g->nodes[$start], $start, $visited, $ll);

  $list->numberArray = LinkedListNumbersToArray($ll);
  FreeLinkedListNumbers($ll);
}
function DepthFirstSearchRecursive($g, $node, $nodeNr, &$visited, $list){

  $visited[$nodeNr] = true;

  LinkedListAddNumber($list, $nodeNr);

  for($i = 0.0; $i < count($node->edge); $i = $i + 1.0){
    $e = $node->edge[$i];
    if( !$visited[$e->nodeNr] ){
      DepthFirstSearchRecursive($g, $g->nodes[$e->nodeNr], $e->nodeNr, $visited, $list);
    }
  }
}
function BreadthFirstSearch($g, $start, $list){

  $da = CreateDynamicArrayNumbers();
  $visited = CreateBooleanArray(count($g->nodes), false);
  $length = 0.0;
  $front = 0.0;

  $visited[$start] = true;

  DynamicArrayAddNumber($da, $start);
  $length = $length + 1.0;

  for(; $front != $length; ){
    $v = DynamicArrayNumbersIndex($da, $front);
    $front = $front + 1.0;

    $n = $g->nodes[$v];

    for($i = 0.0; $i < count($n->edge); $i = $i + 1.0){
      $e = $n->edge[$i];
      if( !$visited[$e->nodeNr] ){
        $visited[$e->nodeNr] = true;

        DynamicArrayAddNumber($da, $e->nodeNr);
        $length = $length + 1.0;
      }
    }
  }

  $list->numberArray = DynamicArrayNumbersToArray($da);
  FreeDynamicArrayNumbers($da);
}
function PrimsAlgorithmNoQueue($g, $forest){

  $valid = DirectedGraphIsValid($g) && IsUndirected($g);

  if($valid){
    $inMST = CreateBooleanArray(count($g->nodes), false);
    $nodesCompleted = 0.0;
    $linkedListTrees = CreateLinkedListNumbersArray(count($g->nodes));
    $roots = CreateLinkedListNumbers();

    for(; $nodesCompleted < count($g->nodes); ){

      /* Find a node not in an MST */
      $found = false;
      $root = 0.0;
      for($i = 0.0; $i < count($g->nodes) &&  !$found ; $i = $i + 1.0){
        if( !$inMST[$i] ){
          $root = $i;
          $found = true;
        }
      }

      LinkedListAddNumber($roots, $root);

      $inMST[$root] = true;
      $nodesCompleted = $nodesCompleted + 1.0;

      $found = true;
      for(; $found; ){
        /* Find minimum edge going out from existing tree. */
        $minimum = 0.0;
        $minimumSet = false;
        $minimumTarget = 0.0;
        $minimumSource = 0.0;
        for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
          if($inMST[$i]){
            $node = $g->nodes[$i];
            for($j = 0.0; $j < count($node->edge); $j = $j + 1.0){
              $edge = $node->edge[$j];
              if( !$inMST[$edge->nodeNr] ){
                if( !$minimumSet ){
                  $minimum = $edge->weight;
                  $minimumTarget = $edge->nodeNr;
                  $minimumSource = $i;
                  $minimumSet = true;
                }else if($edge->weight < $minimum){
                  $minimum = $edge->weight;
                  $minimumTarget = $edge->nodeNr;
                  $minimumSource = $i;
                }
              }
            }
          }
        }

        /* Add edge to tree. */
        if($minimumSet){
          LinkedListAddNumber($linkedListTrees[$minimumSource], $minimumTarget);
          $inMST[$minimumTarget] = true;
          $nodesCompleted = $nodesCompleted + 1.0;
          $found = true;
        }else{
          $found = false;
        }
      }
    }

    ConvertLinkedListTreesToForest($forest, $roots, $linkedListTrees);

    /* Free memory. */
    unset($inMST);
    FreeLinkedListNumbersArray($linkedListTrees);
  }

  return $valid;
}
function PrimsAlgorithm($g, $forest){

  $valid = DirectedGraphIsValid($g) && IsUndirected($g);

  if($valid){
    $q = CreatePriorityQueueBTNumKeyValue();
    $targetReference = new stdClass();
    $weightReference = new stdClass();
    $inMST = CreateBooleanArray(count($g->nodes), false);
    $minimumEdgeSet = CreateBooleanArray(count($g->nodes), false);
    $minimumEdges = CreateNumberArray(count($g->nodes), 0.0);
    $minimumSources = CreateNumberArray(count($g->nodes), 0.0);
    $linkedListTrees = CreateLinkedListNumbersArray(count($g->nodes));
    $roots = CreateLinkedListNumbers();
    $nodesCompleted = 0.0;

    for(; $nodesCompleted < count($g->nodes); ){
      /* Find a node not in an MST */
      $found = false;
      $root = 0.0;
      for($i = 0.0; $i < count($g->nodes) &&  !$found ; $i = $i + 1.0){
        if( !$inMST[$i] ){
          $root = $i;
          $found = true;
        }
      }

      /* Record tree root. */
      LinkedListAddNumber($roots, $root);
      $inMST[$root] = true;
      $nodesCompleted = $nodesCompleted + 1.0;

      /* Add all outgoing edges to priority queue */
      AddOutgoingEdgesToPriorityQueue($g->nodes[$root], $minimumEdgeSet, $root, $minimumEdges, $minimumSources, $q);

      /* Expand tree one vertex at a time. */
      $found = true;
      for(; $found; ){
        /* Find minimum edge going out from existing tree using queue. */
        $minimumSet = false;
        $empty = false;
        for(;  !$minimumSet  &&  !$empty ; ){
          $empty =  !PopPriorityQueueBTNumKeyValue($q, $weightReference, $targetReference) ;
          if( !$empty  &&  !$inMST[$targetReference->numberValue] ){
            $minimumSet = true;
          }
        }

        if($minimumSet){
          /* Add edge to tree. */
          $minimumTarget = $targetReference->numberValue;
          $minimumSource = $minimumSources[$minimumTarget];

          LinkedListAddNumber($linkedListTrees[$minimumSource], $minimumTarget);
          $inMST[$minimumTarget] = true;
          $nodesCompleted = $nodesCompleted + 1.0;
          $found = true;

          /* Add all outgoing edges to priority queue. */
          AddOutgoingEdgesToPriorityQueue($g->nodes[$minimumTarget], $minimumEdgeSet, $minimumTarget, $minimumEdges, $minimumSources, $q);
        }else{
          $found = false;
        }
      }
    }

    ConvertLinkedListTreesToForest($forest, $roots, $linkedListTrees);

    /* Free memory. */
    FreePriorityQueueBTNumKeyValue($q);
    unset($targetReference);
    unset($weightReference);
    unset($inMST);
    unset($minimumEdgeSet);
    FreeLinkedListNumbersArray($linkedListTrees);
    unset($minimumEdges);
    unset($minimumSources);
  }

  return $valid;
}
function AddOutgoingEdgesToPriorityQueue($node, &$minimumEdgeSet, $source, &$minimumEdges, &$minimumSources, $q){

  for($i = 0.0; $i < count($node->edge); $i = $i + 1.0){
    $edge = $node->edge[$i];
    $target = $edge->nodeNr;
    InsertIntoPriorityQueueBTNumKeyValue($q, 1.0/$edge->weight, $target);
    if( !$minimumEdgeSet[$target] ){
      $minimumEdges[$target] = $edge->weight;
      $minimumSources[$target] = $source;
      $minimumEdgeSet[$target] = true;
    }else if($minimumEdges[$target] > $edge->weight){
      $minimumEdges[$target] = $edge->weight;
      $minimumSources[$target] = $source;
    }
  }
}
function KruskalsAlgorithm($g, $forest){

  $valid = DirectedGraphIsValid($g) && IsUndirected($g);

  if($valid){
    $sources = CreateDynamicArrayNumbers();
    $targets = CreateDynamicArrayNumbers();
    $edges = CreateDynamicArrayNumbers();
    $edgeNrReference = new stdClass();
    $weightReference = new stdClass();
    $roots = CreateBooleanArray(count($g->nodes), false);
    $memberOfTree = array_fill(0, count($g->nodes), 0);
    for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
      $memberOfTree[$i] = $i;
    }

    $q = CreatePriorityQueueBTNumKeyValue();

    /* Add all edges to a priority queue. */
    $edgeNr = 0.0;
    for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
      $node = $g->nodes[$i];
      for($j = 0.0; $j < count($node->edge); $j = $j + 1.0){
        $edge = $node->edge[$j];
        InsertIntoPriorityQueueBTNumKeyValue($q, 1.0/$edge->weight, $edgeNr);
        DynamicArrayAddNumber($sources, $i);
        DynamicArrayAddNumber($targets, $edge->nodeNr);

        $edgeNr = $edgeNr + 1.0;
      }
    }

    for(;  !IsEmptyPriorityQueueBTNumKeyValue($q) ; ){
      PopPriorityQueueBTNumKeyValue($q, $weightReference, $edgeNrReference);

      $source = DynamicArrayNumbersIndex($sources, $edgeNrReference->numberValue);
      $target = DynamicArrayNumbersIndex($targets, $edgeNrReference->numberValue);

      if($memberOfTree[$source] != $memberOfTree[$target]){
        $replace = $memberOfTree[$target];
        $replaceWith = $memberOfTree[$source];

        for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
          if($memberOfTree[$i] == $replace){
            $memberOfTree[$i] = $replaceWith;
          }
        }

        DynamicArrayAddNumber($edges, $edgeNrReference->numberValue);
      }
    }

    /* Built forest. */
    $trees = 0.0;
    for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
      $candidate = $memberOfTree[$i];
      if( !$roots[$candidate] ){
        $trees = $trees + 1.0;
        $roots[$candidate] = true;
      }
    }
    $forest->trees = array_fill(0, $trees, 0);
    $treeNr = 0.0;
    for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
      if($roots[$i]){
        $tree = CreateTreeFromEdgeCollection($i, $i, $edges, $sources, $targets);

        $forest->trees[$treeNr] = $tree;
        $treeNr = $treeNr + 1.0;
      }
    }

    /* Free memory. */
    FreePriorityQueueBTNumKeyValue($q);
    FreeDynamicArrayNumbers($sources);
    FreeDynamicArrayNumbers($targets);
    unset($edgeNrReference);
    unset($weightReference);
  }

  return $valid;
}
function CreateTreeFromEdgeCollection($root, $parent, $edges, $sources, $targets){

  $tree = new stdClass();
  $tree->label = $root;
  $branches = CreateLinkedListNumbers();

  $size = 0.0;
  for($i = 0.0; $i < $edges->length; $i = $i + 1.0){
    $edgeNr = DynamicArrayNumbersIndex($edges, $i);

    $source = DynamicArrayNumbersIndex($sources, $edgeNr);
    $target = DynamicArrayNumbersIndex($targets, $edgeNr);

    if($source == $root && $target != $parent){
      LinkedListAddNumber($branches, $target);
      $size = $size + 1.0;
    }else if($target == $root && $source != $parent){
      LinkedListAddNumber($branches, $source);
      $size = $size + 1.0;
    }
  }

  $tree->branches = array_fill(0, $size, 0);
  $node = $branches->first;
  for($i = 0.0; $i < $size; $i = $i + 1.0){
    $tree->branches[$i] = CreateTreeFromEdgeCollection($node->value, $root, $edges, $sources, $targets);

    $node = $node->next;
  }

  return $tree;
}
function DijkstrasAlgorithm($g, $src, $dist, $distSet, $prev){

  $nodes = count($g->nodes);

  $distSet->booleanArray = CreateBooleanArray($nodes, false);
  $nodeDone = CreateBooleanArray($nodes, false);
  $dist->numberArray = CreateNumberArray($nodes, 0.0);
  $prev->numberArray = CreateNumberArray($nodes, 0.0);

  $dist->numberArray[$src] = 0.0;
  $distSet->booleanArray[$src] = true;

  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    /* Get node with lowest distance */
    $u = ListFindLowestSetAndIncluded($dist->numberArray, $distSet->booleanArray, $nodeDone);

    /* Mark node as done */
    $nodeDone[$u] = true;

    $edges = GetEdgesForNodeFromDirectedGraph($g, $u);
    for($j = 0.0; $j < $edges; $j = $j + 1.0){
      $edge = GetEdgeFromDirectedGraph($g, $u, $j);

      if( !$nodeDone[$edge->nodeNr] ){
        $v = $edge->nodeNr;
        $alt = $dist->numberArray[$u] + $edge->weight;
        if( !$distSet->booleanArray[$v] ){
          $dist->numberArray[$v] = $alt;
          $distSet->booleanArray[$v] = true;
          $prev->numberArray[$v] = $u;
        }else if($alt < $dist->numberArray[$v]){
          $dist->numberArray[$v] = $alt;
          $prev->numberArray[$v] = $u;
        }
      }
    }
  }

  unset($distSet);
  unset($nodeDone);
}
function ListFindLowestSetAndIncluded(&$list, &$set, &$exclude){

  $nodes = count($list);
  $lowest = 0.0;
  $u = 0.0;

  $lowestSet = false;
  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    if( !$exclude[$i]  && $set[$i]){
      if( !$lowestSet ){
        $lowest = $list[$i];
        $u = $i;
        $lowestSet = true;
      }else if($list[$i] < $lowest){
        $lowest = $list[$i];
        $u = $i;
      }
    }
  }

  return $u;
}
function DijkstrasAlgorithmDestinationOnly($g, $src, $dest, $path, $distance){

  $distances = new stdClass();
  $previous = new stdClass();
  $distanceSet = new stdClass();

  DijkstrasAlgorithm($g, $src, $distances, $distanceSet, $previous);

  $found = $distanceSet->booleanArray[$dest];

  if($found){
    $distance->numberValue = $distances->numberArray[$dest];

    ExtractForwardPathFromReverseList($src, $dest, $previous, $path);
  }

  unset($distances);
  unset($previous);
  unset($distanceSet);

  return $found;
}
function ExtractForwardPathFromReverseList($src, $dest, $previous, $path){

  $next = $dest;
  for($length = 1.0; $next != $src; $length = $length + 1.0){
    $next = $previous->numberArray[$next];
  }

  $path->numberArray = CreateNumberArray($length, 0.0);

  $next = $dest;
  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $path->numberArray[$length - $i - 1.0] = $next;
    $next = $previous->numberArray[$next];
  }
}
function BellmanFordAlgorithm($g, $src, $dist, $distSet, $prev){

  $nodes = count($g->nodes);

  $distSet->booleanArray = CreateBooleanArray($nodes, false);
  $nodeDone = CreateBooleanArray($nodes, false);
  $dist->numberArray = CreateNumberArray($nodes, 0.0);
  $prev->numberArray = CreateNumberArray($nodes, 0.0);

  $dist->numberArray[$src] = 0.0;
  $distSet->booleanArray[$src] = true;

  for($i = 0.0; $i < $nodes - 1.0; $i = $i + 1.0){
    for($u = 0.0; $u < $nodes; $u = $u + 1.0){
      $edges = GetEdgesForNodeFromDirectedGraph($g, $u);
      for($j = 0.0; $j < $edges; $j = $j + 1.0){
        $edge = GetEdgeFromDirectedGraph($g, $u, $j);

        $v = $edge->nodeNr;
        $w = $edge->weight;

        if($distSet->booleanArray[$u]){
          if( !$distSet->booleanArray[$v] ){
            $dist->numberArray[$v] = $dist->numberArray[$u] + $w;
            $distSet->booleanArray[$v] = true;
            $prev->numberArray[$v] = $u;
          }else if($dist->numberArray[$u] + $w < $dist->numberArray[$v]){
            $dist->numberArray[$v] = $dist->numberArray[$u] + $w;
            $prev->numberArray[$v] = $u;
          }
        }
      }
    }
  }

  $success = true;
  for($u = 0.0; $u < $nodes; $u = $u + 1.0){
    $edges = GetEdgesForNodeFromDirectedGraph($g, $u);
    for($j = 0.0; $j < $edges; $j = $j + 1.0){
      $edge = GetEdgeFromDirectedGraph($g, $u, $j);

      $v = $edge->nodeNr;
      $w = $edge->weight;

      if($dist->numberArray[$u] + $w < $dist->numberArray[$v]){
        $success = false;
      }
    }
  }

  unset($distSet);
  unset($nodeDone);

  return $success;
}
function BellmanFordAlgorithmDestinationOnly($g, $src, $dest, $path, $distance){

  $distances = new stdClass();
  $previous = new stdClass();
  $distanceSet = new stdClass();

  $found = BellmanFordAlgorithm($g, $src, $distances, $distanceSet, $previous);

  if($found){
    $found = $distanceSet->booleanArray[$dest];

    if($found){
      $distance->numberValue = $distances->numberArray[$dest];

      ExtractForwardPathFromReverseList($src, $dest, $previous, $path);
    }
  }

  unset($distances);
  unset($previous);
  unset($distanceSet);

  return $found;
}
function FloydWarshallAlgorithm($g, $distances){

  $success = true;

  for($u = 0.0; $u < count($g->nodes); $u = $u + 1.0){
    $n = $g->nodes[$u];

    for($j = 0.0; $j < count($n->edge); $j = $j + 1.0){
      $e = $n->edge[$j];
      $v = $e->nodeNr;

      $t = $distances->from[$u]->to[$v];
      $t->length = $e->weight;
      $t->lengthSet = true;
      $t->next = $v;
    }
  }

  for($v = 0.0; $v < count($g->nodes); $v = $v + 1.0){
    $t = $distances->from[$v]->to[$v];
    $t->length = 0.0;
    $t->lengthSet = true;
    $t->next = $v;
  }

  for($k = 0.0; $k < count($g->nodes) && $success; $k = $k + 1.0){
    for($i = 0.0; $i < count($g->nodes) && $success; $i = $i + 1.0){
      for($j = 0.0; $j < count($g->nodes) && $success; $j = $j + 1.0){
        $ij = $distances->from[$i]->to[$j];
        $ik = $distances->from[$i]->to[$k];
        $kj = $distances->from[$k]->to[$j];

        if( !$ij->lengthSet  && $ik->lengthSet && $kj->lengthSet){
          $ij->length = $ik->length + $kj->length;
          $ij->lengthSet = true;
          $ij->next = $ik->next;
        }else if($ij->lengthSet && $ik->lengthSet && $kj->lengthSet){
          if($ij->length > $ik->length + $kj->length){
            $ij->length = $ik->length + $kj->length;
            $ij->next = $ik->next;
          }
        }

        if($i == $j){
          if($ij->length < 0.0){
            $success = false;
          }
        }
      }
    }
  }

  return $success;
}
function CreateDistancesFloydWarshallAlgorithm($nodes){

  $distances = new stdClass();
  $distances->from = array_fill(0, $nodes, 0);
  for($i = 0.0; $i < count($distances->from); $i = $i + 1.0){
    $distances->from[$i] = new stdClass();
    $distances->from[$i]->to = array_fill(0, count($distances->from), 0);
    for($j = 0.0; $j < count($distances->from); $j = $j + 1.0){
      $distances->from[$i]->to[$j] = new stdClass();
      $distances->from[$i]->to[$j]->length = 0.0;
      $distances->from[$i]->to[$j]->lengthSet = false;
      $distances->from[$i]->to[$j]->next = 0.0;
    }
  }

  return $distances;
}
function &GetPathFromDistances($distances, $u, $v){

  $t = $distances->from[$u]->to[$v];
  if($t->lengthSet){
    /* count */
    $length = 1.0;
    $next = $u;
    for(; $next != $v; ){
      $next = $distances->from[$next]->to[$v]->next;
      $length = $length + 1.0;
    }

    $path = array_fill(0, $length, 0);

    /* set */
    $next = $u;
    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $path[$i] = $next;
      $next = $distances->from[$next]->to[$v]->next;
    }
  }else{
    $path = array_fill(0, 0.0, 0);
  }

  return $path;
}
function CreateEdge($nodeNr, $weight){

  $e = new stdClass();

  $e->nodeNr = $nodeNr;
  $e->weight = $weight;

  return $e;
}
function DirectedGraphIsValid($g){

  $valid = true;

  for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
    $node = $g->nodes[$i];
    for($j = 0.0; $j < count($node->edge); $j = $j + 1.0){
      $edge = $node->edge[$j];
      if(IsInteger($edge->nodeNr)){
        if($edge->nodeNr >= 0.0 && $edge->nodeNr < count($g->nodes)){
        }else{
          $valid = false;
        }
      }else{
        $valid = false;
      }
    }
  }

  return $valid;
}
function DirectedGraphContainsCycleDFS($g){

  $cycle = false;
  $incoming = DirectedGraphCountIncomingEdges($g);

  $zeroIncomming = 0.0;
  for($i = 0.0; $i < count($g->nodes) &&  !$cycle ; $i = $i + 1.0){
    if($incoming[$i] == 0.0){
      $zeroIncomming = $zeroIncomming + 1.0;

      $cycle = DirectedGraphContainsCycleFromNodeDFS($g, $i);
    }
  }

  unset($incoming);

  if(count($g->nodes) > 0.0 && $zeroIncomming == 0.0){
    $cycle = true;
  }

  return $cycle;
}
function &DirectedGraphCountIncomingEdges($g){

  $incoming = CreateNumberArray(count($g->nodes), 0.0);

  for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
    $node = $g->nodes[$i];
    for($j = 0.0; $j < count($node->edge); $j = $j + 1.0){
      $e = $node->edge[$j];
      $incoming[$e->nodeNr] = $incoming[$e->nodeNr] + 1.0;
    }
  }

  return $incoming;
}
function DirectedGraphContainsCycleFromNodeDFS($g, $nodeNr){

  $visited = CreateBooleanArray(count($g->nodes), false);

  $cycle = DirectedGraphContainsCycleFromNodeDFSRecursive($g, $nodeNr, $visited);

  unset($visited);

  return $cycle;
}
function DirectedGraphContainsCycleFromNodeDFSRecursive($g, $nodeNr, &$visited){

  $cycle = false;
  $node = $g->nodes[$nodeNr];

  for($i = 0.0; $i < count($node->edge) &&  !$cycle ; $i = $i + 1.0){
    $e = $node->edge[$i];
    if($visited[$e->nodeNr]){
      $cycle = true;
    }else{
      $visited[$e->nodeNr] = true;
      $cycle = DirectedGraphContainsCycleFromNodeDFSRecursive($g, $e->nodeNr, $visited);
      $visited[$e->nodeNr] = false;
    }
  }

  return $cycle;
}
function DirectedGraphCountCyclesDFS($g){

  $cycleCount = 0.0;
  $incoming = DirectedGraphCountIncomingEdges($g);

  $zeroIncoming = 0.0;
  for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
    if($incoming[$i] == 0.0){
      $zeroIncoming = $zeroIncoming + 1.0;

      $cycleCount = $cycleCount + DirectedGraphCountCyclesFromNodeDFS($g, $i);
    }
  }

  if(count($g->nodes) > 0.0 && $zeroIncoming == 0.0){
    $cycleCount = $cycleCount + DirectedGraphCountCyclesFromNodeDFS($g, 0.0);
  }

  unset($incoming);

  return $cycleCount;
}
function DirectedGraphCountCyclesFromNodeDFS($g, $nodeNr){

  $color = CreateNumberArray(count($g->nodes), 0.0);

  $cycleCount = DirectedGraphCountCyclesFromNodeDFSRecursive($g, $nodeNr, $color);

  unset($color);

  return $cycleCount;
}
function DirectedGraphCountCyclesFromNodeDFSRecursive($g, $nodeNr, &$color){

  $cycleCount = 0.0;
  $node = $g->nodes[$nodeNr];

  $color[$nodeNr] = 1.0;

  for($i = 0.0; $i < count($node->edge); $i = $i + 1.0){
    $e = $node->edge[$i];

    if($color[$e->nodeNr] != 2.0){
      if($color[$e->nodeNr] == 1.0){
        $cycleCount = $cycleCount + 1.0;
      }else{
        $cycleCount = $cycleCount + DirectedGraphCountCyclesFromNodeDFSRecursive($g, $e->nodeNr, $color);
      }
    }
  }

  $color[$nodeNr] = 2.0;

  return $cycleCount;
}
function &DirectedGraphGetCyclesDFS($g){

  $cycleNumber = CreateNumberReference(0.0);
  $cycleCount = DirectedGraphCountCyclesDFS($g);

  $cycles = array_fill(0, $cycleCount, 0);

  $incoming = DirectedGraphCountIncomingEdges($g);

  $zeroIncoming = 0.0;
  for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
    if($incoming[$i] == 0.0){
      $zeroIncoming = $zeroIncoming + 1.0;

      DirectedGraphGetCyclesFromNodeDFS($g, $i, $cycles, $cycleNumber);
    }
  }

  if(count($g->nodes) > 0.0 && $zeroIncoming == 0.0){
    DirectedGraphGetCyclesFromNodeDFS($g, 0.0, $cycles, $cycleNumber);
  }

  unset($incoming);

  return $cycles;
}
function DirectedGraphGetCyclesFromNodeDFS($g, $nodeNr, &$cycles, $cycleNumber){

  $color = CreateNumberArray(count($g->nodes), 0.0);
  $cycleMark = CreateNumberArray(count($g->nodes), 0.0);
  $previous = CreateNumberArray(count($g->nodes), 0.0);
  $previousLength = 0.0;

  DirectedGraphGetCyclesFromNodeDFSRecursive($g, $nodeNr, $cycleNumber, $color, $cycles, $previous, $previousLength);

  unset($color);
  unset($cycleMark);
}
function DirectedGraphGetCyclesFromNodeDFSRecursive($g, $nodeNr, $cycleNumber, &$color, &$cycles, &$previous, $previousLength){

  $node = $g->nodes[$nodeNr];

  $color[$nodeNr] = 1.0;

  $previous[$previousLength] = $nodeNr;

  for($i = 0.0; $i < count($node->edge); $i = $i + 1.0){
    $e = $node->edge[$i];
    if($color[$e->nodeNr] != 2.0){
      if($color[$e->nodeNr] == 1.0){
        /* Get cycle length */
        $cycleLength = 0.0;
        $done = false;
        $current = $previousLength;
        for(;  !$done ; ){
          $cycleLength = $cycleLength + 1.0;
          if($previous[$current] == $e->nodeNr){
            $done = true;
          }
          $current = $current - 1.0;
        }

        /* Get cycle in order */
        $cycles[$cycleNumber->numberValue] = new stdClass();
        $cycles[$cycleNumber->numberValue]->nodeNrs = array_fill(0, $cycleLength, 0);
        for($j = 0.0; $j < $cycleLength; $j = $j + 1.0){
          $next = $previousLength - $cycleLength + 1.0 + $j;
          $cycles[$cycleNumber->numberValue]->nodeNrs[$j] = $previous[$next];
        }

        $cycleNumber->numberValue = $cycleNumber->numberValue + 1.0;
      }else{
        DirectedGraphGetCyclesFromNodeDFSRecursive($g, $e->nodeNr, $cycleNumber, $color, $cycles, $previous, $previousLength + 1.0);
      }
    }
  }

  $color[$nodeNr] = 2.0;
}
function CreateDirectedGraph($nodes){

  $directedGraph = new stdClass();
  $directedGraph->nodes = array_fill(0, $nodes, 0);

  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    $directedGraph->nodes[$i] = new stdClass();
  }

  return $directedGraph;
}
function CreateDirectedGraphFromMatrixForm($m){

  $nodes = GetNodesFromDirectedGraphMatrix($m);

  $g = CreateDirectedGraph($nodes);

  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    $order = GetNodeOrderFromMatrixForm($m, $i);
    $g->nodes[$i]->edge = array_fill(0, $order, 0);
    $edgeNr = 0.0;
    for($j = 0.0; $j < $nodes; $j = $j + 1.0){
      $edgeValue = GetDirectedGraphMatrixEdge($m, $i, $j);
      if($edgeValue != 0.0){
        $g->nodes[$i]->edge[$edgeNr] = CreateEdge($j, $edgeValue);
        $edgeNr = $edgeNr + 1.0;
      }
    }
  }

  return $g;
}
function GetDirectedGraphMatrixEdge($m, $nodeNr, $edgeNr){
  return $m->c[$nodeNr]->r[$edgeNr];
}
function GetNodeOrderFromMatrixForm($m, $nodeNr){

  $nodes = GetNodesFromDirectedGraphMatrix($m);

  $order = 0.0;
  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    $order = $order + $m->c[$nodeNr]->r[$i];
  }

  return $order;
}
function GetNodesFromDirectedGraphMatrix($m){
  return count($m->c);
}
function CreateDirectedGraphMatrixFromListForm($g){

  $nodes = count($g->nodes);
  $m = CreateDirectedGraphMatrix($nodes);

  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    $node = $g->nodes[$i];
    for($j = 0.0; $j < count($node->edge); $j = $j + 1.0){
      $edge = $node->edge[$j];
      $m->c[$i]->r[$edge->nodeNr] = $edge->weight;
    }
  }

  return $m;
}
function CreateDirectedGraphMatrix($nodes){
  $m = new stdClass();

  $m->c = array_fill(0, $nodes, 0);
  for($i = 0.0; $i < $nodes; $i = $i + 1.0){
    $m->c[$i] = new stdClass();
    $m->c[$i]->r = CreateNumberArray($nodes, 0.0);
  }

  return $m;
}
function DirectedGraphsEqual($a, $b){

  $equal = true;

  if(count($a->nodes) == count($b->nodes)){
    $nodes = count($a->nodes);

    $done = false;
    for($i = 0.0; $i < $nodes &&  !$done ; $i = $i + 1.0){
      if(GetEdgesForNodeFromDirectedGraph($a, $i) == GetEdgesForNodeFromDirectedGraph($b, $i)){
        $edges = GetEdgesForNodeFromDirectedGraph($a, $i);
        $foundCount = 0.0;
        for($j = 0.0; $j < $edges &&  !$done ; $j = $j + 1.0){
          $found = false;
          for($k = 0.0; $k < $edges &&  !$found ; $k = $k + 1.0){
            $edgeA = GetEdgeFromDirectedGraph($a, $i, $j);
            $edgeB = GetEdgeFromDirectedGraph($b, $i, $k);
            if($edgeA->nodeNr == $edgeB->nodeNr){
              if($edgeA->weight == $edgeB->weight){
                $found = true;
              }
            }
          }
          if($found){
            $foundCount = $foundCount + 1.0;
          }else{
            $equal = false;
            $done = true;
          }
        }

        if($foundCount == $edges){
        }else{
          $equal = false;
          $done = true;
        }
      }else{
        $equal = false;
        $done = true;
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function GetEdgesForNodeFromDirectedGraph($g, $nodeNr){
  return count($g->nodes[$nodeNr]->edge);
}
function GetEdgeFromDirectedGraph($g, $nodeNr, $edgeNr){
  return $g->nodes[$nodeNr]->edge[$edgeNr];
}
function DirectedGraphMatricesEqual($a, $b){

  $equal = true;

  if(GetNodesFromDirectedGraphMatrix($a) == GetNodesFromDirectedGraphMatrix($b)){
    $nodes = GetNodesFromDirectedGraphMatrix($a);
    for($i = 0.0; $i < $nodes && $equal; $i = $i + 1.0){
      for($j = 0.0; $j < $nodes && $equal; $j = $j + 1.0){
        if(GetDirectedGraphMatrixEdge($a, $i, $j) == GetDirectedGraphMatrixEdge($b, $i, $j)){
        }else{
          $equal = false;
        }
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function IsUndirected($g){

  $undirected = true;

  for($u = 0.0; $u < count($g->nodes); $u = $u + 1.0){
    $uNode = $g->nodes[$u];
    for($i = 0.0; $i < count($uNode->edge); $i = $i + 1.0){
      $uEdge = $uNode->edge[$i];
      $v = $uEdge->nodeNr;

      if($u == $v){
      }else{
        $vNode = $g->nodes[$v];
        $found = false;
        for($j = 0.0; $j < count($vNode->edge) &&  !$found ; $j = $j + 1.0){
          $vEdge = $vNode->edge[$j];

          if($vEdge->nodeNr == $u && $vEdge->weight == $uEdge->weight){
            $found = true;
          }
        }

        if( !$found ){
          $undirected = false;
        }
      }
    }
  }

  return $undirected;
}
function GetGraphComponents($g, $componentMembership){

  $valid = DirectedGraphIsValid($g) && IsUndirected($g);

  if($valid){
    $componentMembership->numberArray = CreateNumberArray(count($g->nodes), 0.0);
    $componentMembershipSet = CreateBooleanArray(count($g->nodes), false);
    $list = new stdClass();
    $componentNr = 0.0;
    $done = 0.0;
    $startNode = 0.0;

    for(; $done != count($g->nodes); ){
      /* Find a node not currently in a component. */
      $found = false;
      for($i = 0.0; $i < count($g->nodes) &&  !$found ; $i = $i + 1.0){
        if( !$componentMembershipSet[$i] ){
          $startNode = $i;
          $found = true;
        }
      }

      /* Use DFS to find a component */
      DepthFirstSearch($g, $startNode, $list);

      /* Record the component. */
      for($i = 0.0; $i < count($list->numberArray); $i = $i + 1.0){
        $nodeNr = $list->numberArray[$i];
        if( !$componentMembershipSet[$nodeNr] ){
          $componentMembership->numberArray[$nodeNr] = $componentNr;
          $componentMembershipSet[$nodeNr] = true;
          $done = $done + 1.0;
        }
      }

      $componentNr = $componentNr + 1.0;
    }

    unset($componentMembershipSet);
  }

  return $valid;
}
function TopologicalSort($g, $list){

  $valid =  !DirectedGraphContainsCycleDFS($g) ;

  if($valid){

    $visited = CreateBooleanArray(count($g->nodes), false);
    $ll = CreateLinkedListNumbers();

    for($i = 0.0; $i < count($g->nodes); $i = $i + 1.0){
      if( !$visited[$i] ){
        TopologicalSortRecursive($g, $g->nodes[$i], $i, $visited, $ll);
      }
    }

    $list->numberArray = LinkedListNumbersToArray($ll);
    FreeLinkedListNumbers($ll);

    for($i = 0.0; $i < count($list->numberArray)/2.0; $i = $i + 1.0){
      SwapElementsOfArray($list->numberArray, $i, count($list->numberArray) - $i - 1.0);
    }
  }

  return $valid;
}
function TopologicalSortRecursive($g, $node, $nodeNr, &$visited, $list){

  $visited[$nodeNr] = true;

  for($i = 0.0; $i < count($node->edge); $i = $i + 1.0){
    $e = $node->edge[$i];
    if( !$visited[$e->nodeNr] ){
      TopologicalSortRecursive($g, $g->nodes[$e->nodeNr], $e->nodeNr, $visited, $list);
    }
  }

  LinkedListAddNumber($list, $nodeNr);
}
function ConvertPreviousListToForest($forest, &$prev){

  $length = 0.0;
  for($i = 0.0; $i < count($prev); $i = $i + 1.0){
    if($prev[$i] == $i){
      $length = $length + 1.0;
    }
  }

  $forest->trees = array_fill(0, $length, 0);

  $j = 0.0;
  for($i = 0.0; $i < $length; $i = $i + 1.0){
    /* Find next root. */
    $root = 0.0;
    $found = false;
    for(; $j < count($prev) &&  !$found ; $j = $j + 1.0){
      if($prev[$j] == $j){
        $root = $j;
        $found = true;
      }
    }

    /* Create tree from root. */
    $forest->trees[$i] = ConvertPreviousListToTree($root, $prev);
  }
}
function ConvertPreviousListToTree($root, &$prev){

  $tree = new stdClass();
  $tree->label = $root;

  /* Count branches. */
  $branches = 0.0;
  for($i = 0.0; $i < count($prev); $i = $i + 1.0){
    if($prev[$i] == $root && $root != $i){
      $branches = $branches + 1.0;
    }
  }

  /* Add branches. */
  $tree->branches = array_fill(0, $branches, 0);
  $branch = 0.0;
  for($i = 0.0; $i < count($prev); $i = $i + 1.0){
    if($prev[$i] == $root && $root != $i){
      $tree->branches[$branch] = ConvertPreviousListToTree($i, $prev);
      $branch = $branch + 1.0;
    }
  }

  return $tree;
}
function ConvertLinkedListTreesToForest($forest, $roots, &$trees){

  $node = $roots->first;
  $length = LinkedListNumbersLength($roots);
  $forest->trees = array_fill(0, $length, 0);

  for($current = 0.0;  !$node->end ; $current = $current + 1.0){
    $forest->trees[$current] = ConvertLinkedListTreeToTree($node->value, $trees);
    $node = $node->next;
  }
}
function ConvertLinkedListTreeToTree($root, &$trees){

  $rootList = $trees[$root];

  $tree = new stdClass();
  $tree->label = $root;
  $length = LinkedListNumbersLength($rootList);
  $tree->branches = array_fill(0, $length, 0);

  $node = $rootList->first;

  for($current = 0.0;  !$node->end ; $current = $current + 1.0){
    $tree->branches[$current] = ConvertLinkedListTreeToTree($node->value, $trees);
    $node = $node->next;
  }

  return $tree;
}
function SpanningTreeAlgorithmsTest($failures){

  $forest = new stdClass();

  $g = MakeUndirectedGraphForMST();
  $valid = PrimsAlgorithmNoQueue($g, $forest);
  CheckUndirectedGraphForMST($failures, $valid, $forest);
  $valid = PrimsAlgorithm($g, $forest);
  CheckUndirectedGraphForMST($failures, $valid, $forest);
  $valid = KruskalsAlgorithm($g, $forest);
  CheckUndirectedGraphForMSTKruskals($failures, $valid, $forest);

  $g = MakeUndirectedGraph();
  $valid = PrimsAlgorithm($g, $forest);
  CheckUndirectedGraphMST($failures, $valid, $forest);
  $valid = PrimsAlgorithmNoQueue($g, $forest);
  CheckUndirectedGraphMST($failures, $valid, $forest);
  $valid = KruskalsAlgorithm($g, $forest);
  CheckUndirectedGraphMSTKruskals($failures, $valid, $forest);

  $g = MakeUndirectedGraphWithThreeComponents();
  $valid = PrimsAlgorithm($g, $forest);
  CheckUndirectedGraphWithThreeComponentsMST($failures, $valid, $forest);
  $valid = PrimsAlgorithmNoQueue($g, $forest);
  CheckUndirectedGraphWithThreeComponentsMST($failures, $valid, $forest);
  $valid = KruskalsAlgorithm($g, $forest);
  CheckUndirectedGraphWithThreeComponentsMSTKruskals($failures, $valid, $forest);

  $g = MakeUndirectedGraphForMST2();
  $valid = PrimsAlgorithmNoQueue($g, $forest);
  /*CheckUndirectedGraphForMST2(failures, valid, forest); */
  $valid = PrimsAlgorithm($g, $forest);
  CheckUndirectedGraphForMST2($failures, $valid, $forest);
  $valid = KruskalsAlgorithm($g, $forest);
  CheckUndirectedGraphForMST2Kruskals($failures, $valid, $forest);
}
function CheckUndirectedGraphWithThreeComponentsMST($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 3.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[2.0]->label, 4.0, $failures);
  AssertEquals($forest->trees[2.0]->branches[0.0]->label, 5.0, $failures);
  AssertEquals($forest->trees[2.0]->branches[0.0]->branches[0.0]->label, 6.0, $failures);
}
function CheckUndirectedGraphWithThreeComponentsMSTKruskals($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 3.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[2.0]->label, 6.0, $failures);
  AssertEquals($forest->trees[2.0]->branches[0.0]->label, 5.0, $failures);
  AssertEquals($forest->trees[2.0]->branches[0.0]->branches[0.0]->label, 4.0, $failures);
}
function CheckUndirectedGraphMST($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 1.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 2.0, $failures);
}
function CheckUndirectedGraphMSTKruskals($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 1.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 0.0, $failures);
}
function CheckUndirectedGraphForMST($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 4.0, $failures);
  AssertEquals(count($forest->trees[1.0]->branches), 0.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals(count($forest->trees[0.0]->branches), 2.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[1.0]->label, 1.0, $failures);
}
function CheckUndirectedGraphForMST2($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 4.0, $failures);
  AssertEquals(count($forest->trees[1.0]->branches), 0.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[1.0]->label, 1.0, $failures);
}
function CheckUndirectedGraphForMSTKruskals($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 4.0, $failures);
  AssertEquals(count($forest->trees[1.0]->branches), 0.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 0.0, $failures);
  AssertEquals(count($forest->trees[0.0]->branches), 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 1.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[1.0]->label, 2.0, $failures);
}
function CheckUndirectedGraphForMST2Kruskals($failures, $valid, $forest){
  AssertTrue($valid, $failures);
  AssertEquals(count($forest->trees), 2.0, $failures);
  AssertEquals($forest->trees[1.0]->label, 4.0, $failures);
  AssertEquals(count($forest->trees[1.0]->branches), 0.0, $failures);
  AssertEquals($forest->trees[0.0]->label, 2.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->label, 3.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->label, 0.0, $failures);
  AssertEquals($forest->trees[0.0]->branches[0.0]->branches[0.0]->branches[0.0]->label, 1.0, $failures);
}
function searchTests($failures){
  DFSTest1($failures);
  DFSTest2($failures);
  DFSTest3($failures);

  BFSTest1($failures);
  BFSTest2($failures);
  BFSTest3($failures);
}
function DFSTest1($failures){

  $directedGraph = MakeGraphWithTwoMixedCycles();

  $list = new stdClass();

  DepthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 2.0, $failures);
  AssertEquals($list->numberArray[3.0], 3.0, $failures);
}
function DFSTest2($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $list = new stdClass();

  DepthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 2.0, $failures);
  AssertEquals($list->numberArray[3.0], 3.0, $failures);
  AssertEquals($list->numberArray[4.0], 4.0, $failures);
  AssertEquals($list->numberArray[5.0], 5.0, $failures);
}
function DFSTest3($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm2();

  $list = CreateNumberArrayReferenceLengthValue(count($directedGraph->nodes), 0.0);

  DepthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 7.0, $failures);
  AssertEquals($list->numberArray[3.0], 2.0, $failures);
  AssertEquals($list->numberArray[4.0], 8.0, $failures);
  AssertEquals($list->numberArray[5.0], 3.0, $failures);
  AssertEquals($list->numberArray[6.0], 4.0, $failures);
  AssertEquals($list->numberArray[7.0], 5.0, $failures);
  AssertEquals($list->numberArray[8.0], 6.0, $failures);
}
function BFSTest1($failures){

  $directedGraph = MakeGraphWithTwoMixedCycles();

  $list = new stdClass();

  BreadthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 2.0, $failures);
  AssertEquals($list->numberArray[3.0], 3.0, $failures);
}
function BFSTest2($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $list = new stdClass();

  BreadthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 2.0, $failures);
  AssertEquals($list->numberArray[3.0], 5.0, $failures);
  AssertEquals($list->numberArray[4.0], 3.0, $failures);
  AssertEquals($list->numberArray[5.0], 4.0, $failures);
}
function BFSTest3($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm2();

  $list = new stdClass();

  BreadthFirstSearch($directedGraph, 0.0, $list);

  AssertEquals($list->numberArray[0.0], 0.0, $failures);
  AssertEquals($list->numberArray[1.0], 1.0, $failures);
  AssertEquals($list->numberArray[2.0], 7.0, $failures);
  AssertEquals($list->numberArray[3.0], 6.0, $failures);
  AssertEquals($list->numberArray[4.0], 2.0, $failures);
  AssertEquals($list->numberArray[5.0], 8.0, $failures);
  AssertEquals($list->numberArray[6.0], 5.0, $failures);
  AssertEquals($list->numberArray[7.0], 3.0, $failures);
  AssertEquals($list->numberArray[8.0], 4.0, $failures);
}
function ShortestPathsTests($failures){
  testDijkstrasAlgorithm($failures);
  testDijkstrasAlgorithm2($failures);
  testBellmanFordAlgorithm($failures);
  testBellmanFordAlgorithm2($failures);
  testFloydWarshallAlgorithm($failures);
  testFloydWarshallAlgorithm2($failures);
  testShortestPathAlgorithmsDistanceOnly($failures);
}
function testDijkstrasAlgorithm($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $d = new stdClass();
  $p = new stdClass();
  $ds = new stdClass();
  DijkstrasAlgorithm($directedGraph, 0.0, $d, $ds, $p);

  CheckShortestPath($failures, $d, $p);
}
function testShortestPathAlgorithmsDistanceOnly($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $path = new stdClass();
  $distance = new stdClass();
  $success = DijkstrasAlgorithmDestinationOnly($directedGraph, 0.0, 5.0, $path, $distance);

  AssertTrue($success, $failures);
  AssertEquals($distance->numberValue, 11.0, $failures);
  AssertEquals(count($path->numberArray), 3.0, $failures);
  AssertEquals($path->numberArray[0.0], 0.0, $failures);
  AssertEquals($path->numberArray[1.0], 2.0, $failures);
  AssertEquals($path->numberArray[2.0], 5.0, $failures);

  $path = new stdClass();
  $distance = new stdClass();
  $success = BellmanFordAlgorithmDestinationOnly($directedGraph, 0.0, 5.0, $path, $distance);

  AssertTrue($success, $failures);
  AssertEquals($distance->numberValue, 11.0, $failures);
  AssertEquals(count($path->numberArray), 3.0, $failures);
  AssertEquals($path->numberArray[0.0], 0.0, $failures);
  AssertEquals($path->numberArray[1.0], 2.0, $failures);
  AssertEquals($path->numberArray[2.0], 5.0, $failures);
}
function testBellmanFordAlgorithm($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $d = new stdClass();
  $p = new stdClass();
  $ds = new stdClass();
  $success = BellmanFordAlgorithm($directedGraph, 0.0, $d, $ds, $p);

  AssertTrue($success, $failures);
  CheckShortestPath($failures, $d, $p);
}
function CheckShortestPath($failures, $d, $p){
  AssertEquals($d->numberArray[0.0], 0.0, $failures);
  AssertEquals($d->numberArray[1.0], 7.0, $failures);
  AssertEquals($d->numberArray[2.0], 9.0, $failures);
  AssertEquals($d->numberArray[3.0], 20.0, $failures);
  AssertEquals($d->numberArray[4.0], 20.0, $failures);
  AssertEquals($d->numberArray[5.0], 11.0, $failures);

  AssertEquals($p->numberArray[0.0], 0.0, $failures);
  AssertEquals($p->numberArray[1.0], 0.0, $failures);
  AssertEquals($p->numberArray[2.0], 0.0, $failures);
  AssertEquals($p->numberArray[3.0], 2.0, $failures);
  AssertEquals($p->numberArray[4.0], 5.0, $failures);
  AssertEquals($p->numberArray[5.0], 2.0, $failures);
}
function MakeGraphForDijkstrasAlgorithm(){

  $directedGraph = CreateDirectedGraph(6.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 7.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(2.0, 9.0);
  $directedGraph->nodes[0.0]->edge[2.0] = CreateEdge(5.0, 14.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 7.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(2.0, 10.0);
  $directedGraph->nodes[1.0]->edge[2.0] = CreateEdge(3.0, 15.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 4.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(0.0, 9.0);
  $directedGraph->nodes[2.0]->edge[1.0] = CreateEdge(1.0, 10.0);
  $directedGraph->nodes[2.0]->edge[2.0] = CreateEdge(3.0, 11.0);
  $directedGraph->nodes[2.0]->edge[3.0] = CreateEdge(5.0, 2.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(1.0, 15.0);
  $directedGraph->nodes[3.0]->edge[1.0] = CreateEdge(2.0, 11.0);
  $directedGraph->nodes[3.0]->edge[2.0] = CreateEdge(4.0, 6.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[4.0]->edge[0.0] = CreateEdge(3.0, 6.0);
  $directedGraph->nodes[4.0]->edge[1.0] = CreateEdge(5.0, 9.0);

  $directedGraph->nodes[5.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[5.0]->edge[0.0] = CreateEdge(0.0, 14.0);
  $directedGraph->nodes[5.0]->edge[1.0] = CreateEdge(2.0, 2.0);
  $directedGraph->nodes[5.0]->edge[2.0] = CreateEdge(4.0, 9.0);

  return $directedGraph;
}
function testDijkstrasAlgorithm2($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm2();
  $d = new stdClass();
  $p = new stdClass();
  $ds = new stdClass();
  DijkstrasAlgorithm($directedGraph, 0.0, $d, $ds, $p);

  CheckShortestPath2($failures, $d, $p);
}
function testBellmanFordAlgorithm2($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm2();
  $d = new stdClass();
  $p = new stdClass();
  $ds = new stdClass();
  BellmanFordAlgorithm($directedGraph, 0.0, $d, $ds, $p);

  CheckShortestPath2($failures, $d, $p);
}
function CheckShortestPath2($failures, $d, $p){
  AssertEquals($d->numberArray[0.0], 0.0, $failures);
  AssertEquals($d->numberArray[1.0], 5.0, $failures);
  AssertEquals($d->numberArray[2.0], 7.0, $failures);
  AssertEquals($d->numberArray[3.0], 7.0, $failures);
  AssertEquals($d->numberArray[4.0], 7.0, $failures);
  AssertEquals($d->numberArray[5.0], 6.0, $failures);
  AssertEquals($d->numberArray[6.0], 3.0, $failures);
  AssertEquals($d->numberArray[7.0], 4.0, $failures);
  AssertEquals($d->numberArray[8.0], 5.0, $failures);

  AssertEquals($p->numberArray[0.0], 0.0, $failures);
  AssertEquals($p->numberArray[1.0], 0.0, $failures);
  AssertTrue($p->numberArray[2.0] == 7.0 || $p->numberArray[2.0] == 1.0, $failures);
  AssertEquals($p->numberArray[3.0], 8.0, $failures);
  AssertEquals($p->numberArray[4.0], 5.0, $failures);
  AssertEquals($p->numberArray[5.0], 7.0, $failures);
  AssertEquals($p->numberArray[6.0], 0.0, $failures);
  AssertEquals($p->numberArray[7.0], 6.0, $failures);
  AssertEquals($p->numberArray[8.0], 7.0, $failures);
}
function MakeGraphForDijkstrasAlgorithm2(){

  $directedGraph = CreateDirectedGraph(9.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 5.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(7.0, 7.0);
  $directedGraph->nodes[0.0]->edge[2.0] = CreateEdge(6.0, 3.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 5.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(7.0, 3.0);
  $directedGraph->nodes[1.0]->edge[2.0] = CreateEdge(2.0, 2.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 4.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(1.0, 2.0);
  $directedGraph->nodes[2.0]->edge[1.0] = CreateEdge(7.0, 3.0);
  $directedGraph->nodes[2.0]->edge[2.0] = CreateEdge(8.0, 3.0);
  $directedGraph->nodes[2.0]->edge[3.0] = CreateEdge(3.0, 4.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(2.0, 4.0);
  $directedGraph->nodes[3.0]->edge[1.0] = CreateEdge(8.0, 2.0);
  $directedGraph->nodes[3.0]->edge[2.0] = CreateEdge(4.0, 5.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[4.0]->edge[0.0] = CreateEdge(3.0, 5.0);
  $directedGraph->nodes[4.0]->edge[1.0] = CreateEdge(8.0, 3.0);
  $directedGraph->nodes[4.0]->edge[2.0] = CreateEdge(5.0, 1.0);

  $directedGraph->nodes[5.0]->edge = array_fill(0, 4.0, 0);
  $directedGraph->nodes[5.0]->edge[0.0] = CreateEdge(4.0, 1.0);
  $directedGraph->nodes[5.0]->edge[1.0] = CreateEdge(8.0, 2.0);
  $directedGraph->nodes[5.0]->edge[2.0] = CreateEdge(7.0, 2.0);
  $directedGraph->nodes[5.0]->edge[3.0] = CreateEdge(6.0, 7.0);

  $directedGraph->nodes[6.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[6.0]->edge[0.0] = CreateEdge(0.0, 3.0);
  $directedGraph->nodes[6.0]->edge[1.0] = CreateEdge(7.0, 1.0);
  $directedGraph->nodes[6.0]->edge[2.0] = CreateEdge(5.0, 7.0);

  $directedGraph->nodes[7.0]->edge = array_fill(0, 6.0, 0);
  $directedGraph->nodes[7.0]->edge[0.0] = CreateEdge(0.0, 7.0);
  $directedGraph->nodes[7.0]->edge[1.0] = CreateEdge(1.0, 3.0);
  $directedGraph->nodes[7.0]->edge[2.0] = CreateEdge(2.0, 3.0);
  $directedGraph->nodes[7.0]->edge[3.0] = CreateEdge(8.0, 1.0);
  $directedGraph->nodes[7.0]->edge[4.0] = CreateEdge(5.0, 2.0);
  $directedGraph->nodes[7.0]->edge[5.0] = CreateEdge(6.0, 1.0);

  $directedGraph->nodes[8.0]->edge = array_fill(0, 5.0, 0);
  $directedGraph->nodes[8.0]->edge[0.0] = CreateEdge(3.0, 2.0);
  $directedGraph->nodes[8.0]->edge[1.0] = CreateEdge(2.0, 3.0);
  $directedGraph->nodes[8.0]->edge[2.0] = CreateEdge(7.0, 1.0);
  $directedGraph->nodes[8.0]->edge[3.0] = CreateEdge(5.0, 2.0);
  $directedGraph->nodes[8.0]->edge[4.0] = CreateEdge(4.0, 3.0);

  return $directedGraph;
}
function testFloydWarshallAlgorithm($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm();

  $distances = CreateDistancesFloydWarshallAlgorithm(count($directedGraph->nodes));
  $success = FloydWarshallAlgorithm($directedGraph, $distances);

  AssertTrue($success, $failures);

  $d = CreateNumberArrayReferenceLengthValue(count($directedGraph->nodes), 0.0);
  $p = CreateNumberArrayReferenceLengthValue(count($directedGraph->nodes), 0.0);

  for($i = 0.0; $i < count($directedGraph->nodes); $i = $i + 1.0){
    $d->numberArray[$i] = $distances->from[0.0]->to[$i]->length;
    $path = GetPathFromDistances($distances, 0.0, $i);
    if(count($path) >= 2.0){
      $p->numberArray[$i] = $path[count($path) - 2.0];
    }else{
      $p->numberArray[$i] = $i;
    }
  }

  CheckShortestPath($failures, $d, $p);
}
function testFloydWarshallAlgorithm2($failures){

  $directedGraph = MakeGraphForDijkstrasAlgorithm2();

  $distances = CreateDistancesFloydWarshallAlgorithm(count($directedGraph->nodes));
  $success = FloydWarshallAlgorithm($directedGraph, $distances);

  AssertTrue($success, $failures);

  $d = CreateNumberArrayReferenceLengthValue(count($directedGraph->nodes), 0.0);
  $p = CreateNumberArrayReferenceLengthValue(count($directedGraph->nodes), 0.0);

  for($i = 0.0; $i < count($directedGraph->nodes); $i = $i + 1.0){
    $d->numberArray[$i] = $distances->from[0.0]->to[$i]->length;
    $path = GetPathFromDistances($distances, 0.0, $i);
    if(count($path) >= 2.0){
      $p->numberArray[$i] = $path[count($path) - 2.0];
    }else{
      $p->numberArray[$i] = $i;
    }
  }

  CheckShortestPath2($failures, $d, $p);
}
function test(){

  $failures = CreateNumberReference(0.0);

  testOneCycle($failures);
  testNoCycle($failures);
  testTwoCycles($failures);
  testOneSelfCycle($failures);
  testTwoMixedCycles($failures);
  testMatrixFormConversions($failures);
  ShortestPathsTests($failures);
  searchTests($failures);
  IsUndirectedTests($failures);
  GraphComponentsTest($failures);
  SpanningTreeAlgorithmsTest($failures);
  TestTopologicalSort($failures);

  return $failures->numberValue;
}
function testOneSelfCycle($failures){

  $directedGraph = MakeGraphWithOneSelfCycle();
  $valid = DirectedGraphIsValid($directedGraph);
  AssertTrue($valid, $failures);
  $cycle = DirectedGraphContainsCycleDFS($directedGraph);
  AssertTrue($cycle, $failures);
  $cycleCount = DirectedGraphCountCyclesDFS($directedGraph);
  AssertEquals(1.0, $cycleCount, $failures);
  $cycles = DirectedGraphGetCyclesDFS($directedGraph);
  AssertEquals(1.0, count($cycles), $failures);
  AssertEquals(1.0, count($cycles[0.0]->nodeNrs), $failures);
  AssertEquals(2.0, $cycles[0.0]->nodeNrs[0.0], $failures);
}
function testTwoCycles($failures){

  $directedGraph = MakeGraphWithTwoCycles();
  $valid = DirectedGraphIsValid($directedGraph);
  AssertTrue($valid, $failures);
  $cycle = DirectedGraphContainsCycleDFS($directedGraph);
  AssertTrue($cycle, $failures);
  $cycleCount = DirectedGraphCountCyclesDFS($directedGraph);
  AssertEquals(2.0, $cycleCount, $failures);
  $cycles = DirectedGraphGetCyclesDFS($directedGraph);
  AssertEquals(2.0, count($cycles), $failures);
  AssertEquals(3.0, count($cycles[0.0]->nodeNrs), $failures);
  AssertEquals(1.0, $cycles[0.0]->nodeNrs[0.0], $failures);
  AssertEquals(2.0, $cycles[0.0]->nodeNrs[1.0], $failures);
  AssertEquals(3.0, $cycles[0.0]->nodeNrs[2.0], $failures);
  AssertEquals(2.0, count($cycles[1.0]->nodeNrs), $failures);
  AssertEquals(0.0, $cycles[1.0]->nodeNrs[0.0], $failures);
  AssertEquals(4.0, $cycles[1.0]->nodeNrs[1.0], $failures);
}
function testNoCycle($failures){

  $directedGraph = MakeGraphWithoutCycle();
  $valid = DirectedGraphIsValid($directedGraph);
  AssertTrue($valid, $failures);
  $cycle = DirectedGraphContainsCycleDFS($directedGraph);
  AssertFalse($cycle, $failures);
  $cycleCount = DirectedGraphCountCyclesDFS($directedGraph);
  AssertEquals(0.0, $cycleCount, $failures);
  $cycles = DirectedGraphGetCyclesDFS($directedGraph);
  AssertEquals(0.0, count($cycles), $failures);
}
function testOneCycle($failures){

  $directedGraph = MakeGraphWithOneCycle();
  $valid = DirectedGraphIsValid($directedGraph);
  AssertTrue($valid, $failures);
  $cycle = DirectedGraphContainsCycleDFS($directedGraph);
  AssertTrue($cycle, $failures);
  $cycleCount = DirectedGraphCountCyclesDFS($directedGraph);
  AssertEquals(1.0, $cycleCount, $failures);
  $cycles = DirectedGraphGetCyclesDFS($directedGraph);
  AssertEquals(1.0, count($cycles), $failures);
  AssertEquals(3.0, count($cycles[0.0]->nodeNrs), $failures);
  AssertEquals(1.0, $cycles[0.0]->nodeNrs[0.0], $failures);
  AssertEquals(2.0, $cycles[0.0]->nodeNrs[1.0], $failures);
  AssertEquals(3.0, $cycles[0.0]->nodeNrs[2.0], $failures);
}
function testTwoMixedCycles($failures){

  $directedGraph = MakeGraphWithTwoMixedCycles();
  $valid = DirectedGraphIsValid($directedGraph);
  AssertTrue($valid, $failures);
  $cycle = DirectedGraphContainsCycleDFS($directedGraph);
  AssertTrue($cycle, $failures);
  $cycleCount = DirectedGraphCountCyclesDFS($directedGraph);
  AssertEquals(2.0, $cycleCount, $failures);
  $cycles = DirectedGraphGetCyclesDFS($directedGraph);
  AssertEquals(2.0, count($cycles), $failures);
  AssertEquals(3.0, count($cycles[0.0]->nodeNrs), $failures);
  AssertEquals(1.0, $cycles[0.0]->nodeNrs[0.0], $failures);
  AssertEquals(2.0, $cycles[0.0]->nodeNrs[1.0], $failures);
  AssertEquals(3.0, $cycles[0.0]->nodeNrs[2.0], $failures);
  AssertEquals(1.0, count($cycles[1.0]->nodeNrs), $failures);
  AssertEquals(3.0, $cycles[1.0]->nodeNrs[0.0], $failures);
}
function testMatrixFormConversions($failures){

  $a = MakeGraphWithTwoMixedCycles();
  $b = CreateDirectedGraphFromMatrixForm(CreateDirectedGraphMatrixFromListForm(MakeGraphWithTwoMixedCycles()));

  AssertTrue(DirectedGraphsEqual($a, $b), $failures);

  $m1 = CreateDirectedGraphMatrixFromListForm($a);
  $m2 = CreateDirectedGraphMatrixFromListForm($a);

  AssertTrue(DirectedGraphMatricesEqual($m1, $m2), $failures);
}
function IsUndirectedTests($failures){

  $g = MakeGraphWithTwoMixedCycles();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeGraphWithOneSelfCycle();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeGraphWithOneCycle();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeGraphWithoutCycle();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeGraphWithTwoCycles();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeGraphWithTwoMixedCycles();

  $undirected = IsUndirected($g);
  AssertFalse($undirected, $failures);

  $g = MakeUndirectedGraph();

  $undirected = IsUndirected($g);
  AssertTrue($undirected, $failures);

  $g = MakeUndirectedGraphForMST();

  $undirected = IsUndirected($g);
  AssertTrue($undirected, $failures);
}
function GraphComponentsTest($failures){

  $g = MakeUndirectedGraph();

  $components = new stdClass();
  $valid = GetGraphComponents($g, $components);
  AssertTrue($valid, $failures);

  AssertEquals($components->numberArray[0.0], 0.0, $failures);
  AssertEquals($components->numberArray[1.0], 0.0, $failures);
  AssertEquals($components->numberArray[2.0], 0.0, $failures);

  $g = MakeUndirectedGraphWithThreeComponents();

  $components = new stdClass();
  $valid = GetGraphComponents($g, $components);
  AssertTrue($valid, $failures);

  AssertEquals($components->numberArray[0.0], 0.0, $failures);
  AssertEquals($components->numberArray[1.0], 0.0, $failures);
  AssertEquals($components->numberArray[2.0], 0.0, $failures);
  AssertEquals($components->numberArray[3.0], 1.0, $failures);
  AssertEquals($components->numberArray[4.0], 2.0, $failures);
  AssertEquals($components->numberArray[5.0], 2.0, $failures);
  AssertEquals($components->numberArray[6.0], 2.0, $failures);

  FreeNumberArrayReference($components);
}
function TestTopologicalSort($failures){

  $g = MakeTopologicalSortGraph();
  $list = new stdClass();

  $valid = TopologicalSort($g, $list);

  AssertTrue($valid, $failures);
  AssertEquals($list->numberArray[0.0], 5.0, $failures);
  AssertEquals($list->numberArray[1.0], 4.0, $failures);
  AssertEquals($list->numberArray[2.0], 2.0, $failures);
  AssertEquals($list->numberArray[3.0], 3.0, $failures);
  AssertEquals($list->numberArray[4.0], 1.0, $failures);
  AssertEquals($list->numberArray[5.0], 0.0, $failures);
}
function MakeGraphWithOneSelfCycle(){
  $directedGraph = CreateDirectedGraph(4.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(2.0, 1.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 0.0, 0);

  return $directedGraph;
}
function MakeGraphWithOneCycle(){
  $directedGraph = CreateDirectedGraph(4.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  return $directedGraph;
}
function MakeGraphWithoutCycle(){
  $directedGraph = CreateDirectedGraph(3.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 0.0, 0);

  return $directedGraph;
}
function MakeGraphWithTwoCycles(){
  $directedGraph = CreateDirectedGraph(5.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(4.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[4.0]->edge[0.0] = CreateEdge(0.0, 1.0);

  return $directedGraph;
}
function MakeGraphWithTwoMixedCycles(){
  $directedGraph = CreateDirectedGraph(4.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(1.0, 1.0);
  $directedGraph->nodes[3.0]->edge[1.0] = CreateEdge(3.0, 1.0);

  return $directedGraph;
}
function MakeUndirectedGraph(){
  $directedGraph = CreateDirectedGraph(3.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 1.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(0.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 1.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  return $directedGraph;
}
function MakeUndirectedGraphWithThreeComponents(){
  $directedGraph = CreateDirectedGraph(7.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 2.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(0.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 2.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(2.0, 1.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 0.0, 0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[4.0]->edge[0.0] = CreateEdge(5.0, 1.0);
  $directedGraph->nodes[4.0]->edge[1.0] = CreateEdge(4.0, 1.0);

  $directedGraph->nodes[5.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[5.0]->edge[0.0] = CreateEdge(4.0, 1.0);
  $directedGraph->nodes[5.0]->edge[1.0] = CreateEdge(6.0, 1.0);

  $directedGraph->nodes[6.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[6.0]->edge[0.0] = CreateEdge(5.0, 1.0);

  return $directedGraph;
}
function MakeUndirectedGraphForMST(){
  $directedGraph = CreateDirectedGraph(5.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 2.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 2.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(3.0, 2.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 3.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(0.0, 1.0);
  $directedGraph->nodes[3.0]->edge[1.0] = CreateEdge(1.0, 2.0);
  $directedGraph->nodes[3.0]->edge[2.0] = CreateEdge(2.0, 3.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 0.0, 0);

  return $directedGraph;
}
function MakeUndirectedGraphForMST2(){
  $directedGraph = CreateDirectedGraph(5.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[0.0]->edge[0.0] = CreateEdge(1.0, 2.0);
  $directedGraph->nodes[0.0]->edge[1.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[1.0]->edge[0.0] = CreateEdge(0.0, 2.0);
  $directedGraph->nodes[1.0]->edge[1.0] = CreateEdge(3.0, 4.0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 3.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 3.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(0.0, 1.0);
  $directedGraph->nodes[3.0]->edge[1.0] = CreateEdge(1.0, 4.0);
  $directedGraph->nodes[3.0]->edge[2.0] = CreateEdge(2.0, 3.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 0.0, 0);

  return $directedGraph;
}
function MakeTopologicalSortGraph(){
  $directedGraph = CreateDirectedGraph(6.0);

  $directedGraph->nodes[0.0]->edge = array_fill(0, 0.0, 0);

  $directedGraph->nodes[1.0]->edge = array_fill(0, 0.0, 0);

  $directedGraph->nodes[2.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[2.0]->edge[0.0] = CreateEdge(3.0, 1.0);

  $directedGraph->nodes[3.0]->edge = array_fill(0, 1.0, 0);
  $directedGraph->nodes[3.0]->edge[0.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[4.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[4.0]->edge[0.0] = CreateEdge(0.0, 1.0);
  $directedGraph->nodes[4.0]->edge[1.0] = CreateEdge(1.0, 1.0);

  $directedGraph->nodes[5.0]->edge = array_fill(0, 2.0, 0);
  $directedGraph->nodes[5.0]->edge[0.0] = CreateEdge(0.0, 1.0);
  $directedGraph->nodes[5.0]->edge[1.0] = CreateEdge(2.0, 1.0);

  return $directedGraph;
}
function &StringToNumberArray(&$string){

  $array = array_fill(0, count($string), 0);

  for($i = 0.0; $i < count($string); $i = $i + 1.0){
    $array[$i] = uniord($string[$i]);
  }
  return $array;
}
function &NumberArrayToString(&$array){

  $string = array_fill(0, count($array), 0);

  for($i = 0.0; $i < count($array); $i = $i + 1.0){
    $string[$i] = unichr($array[$i]);
  }
  return $string;
}
function NumberArraysEqual(&$a, &$b){

  $equal = true;
  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a) && $equal; $i = $i + 1.0){
      if($a[$i] != $b[$i]){
        $equal = false;
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function BooleanArraysEqual(&$a, &$b){

  $equal = true;
  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a) && $equal; $i = $i + 1.0){
      if($a[$i] != $b[$i]){
        $equal = false;
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function StringsEqual(&$a, &$b){

  $equal = true;
  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a) && $equal; $i = $i + 1.0){
      if($a[$i] != $b[$i]){
        $equal = false;
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function FillNumberArray(&$a, $value){

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $a[$i] = $value;
  }
}
function FillString(&$a, $value){

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $a[$i] = $value;
  }
}
function FillBooleanArray(&$a, $value){

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $a[$i] = $value;
  }
}
function FillNumberArrayRange(&$a, $value, $from, $to){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $a[$from + $i] = $value;
    }

    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function FillBooleanArrayRange(&$a, $value, $from, $to){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $a[$from + $i] = $value;
    }

    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function FillStringRange(&$a, $value, $from, $to){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $a[$from + $i] = $value;
    }

    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function &CopyNumberArray(&$a){

  $n = array_fill(0, count($a), 0);

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $n[$i] = $a[$i];
  }

  return $n;
}
function &CopyBooleanArray(&$a){

  $n = array_fill(0, count($a), 0);

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $n[$i] = $a[$i];
  }

  return $n;
}
function &CopyString(&$a){

  $n = array_fill(0, count($a), 0);

  for($i = 0.0; $i < count($a); $i = $i + 1.0){
    $n[$i] = $a[$i];
  }

  return $n;
}
function CopyNumberArrayRange(&$a, $from, $to, $copyReference){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    $n = array_fill(0, $length, 0);

    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $n[$i] = $a[$from + $i];
    }

    $copyReference->numberArray = $n;
    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function CopyBooleanArrayRange(&$a, $from, $to, $copyReference){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    $n = array_fill(0, $length, 0);

    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $n[$i] = $a[$from + $i];
    }

    $copyReference->booleanArray = $n;
    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function CopyStringRange(&$a, $from, $to, $copyReference){

  if($from >= 0.0 && $from <= count($a) && $to >= 0.0 && $to <= count($a) && $from <= $to){
    $length = $to - $from;
    $n = array_fill(0, $length, 0);

    for($i = 0.0; $i < $length; $i = $i + 1.0){
      $n[$i] = $a[$from + $i];
    }

    $copyReference->string = $n;
    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function IsLastElement($length, $index){
  return $index + 1.0 == $length;
}
function &CreateNumberArray($length, $value){

  $array = array_fill(0, $length, 0);
  FillNumberArray($array, $value);

  return $array;
}
function &CreateBooleanArray($length, $value){

  $array = array_fill(0, $length, 0);
  FillBooleanArray($array, $value);

  return $array;
}
function &CreateString($length, $value){

  $array = array_fill(0, $length, 0);
  FillString($array, $value);

  return $array;
}
function SwapElementsOfArray(&$A, $ai, $bi){

  $tmp = $A[$ai];
  $A[$ai] = $A[$bi];
  $A[$bi] = $tmp;
}
function Negate($x){
  return -$x;
}
function Positive($x){
  return +$x;
}
function Factorial($x){
  $f = 1.0;

  for($i = 2.0; $i <= $x; $i = $i + 1.0){
    $f = $f*$i;
  }

  return $f;
}
function Roundx($x){
  return floor($x + 0.5);
}
function BankersRound($x){
  if(Absolute($x - Truncate($x)) == 0.5){
    if( !DivisibleBy(Roundx($x), 2.0) ){
      $r = Roundx($x) - 1.0;
    }else{
      $r = Roundx($x);
    }
  }else{
    $r = Roundx($x);
  }

  return $r;
}
function Ceilx($x){
  return ceil($x);
}
function Floorx($x){
  return floor($x);
}
function Truncate($x){
  if($x >= 0.0){
    $t = floor($x);
  }else{
    $t = ceil($x);
  }

  return $t;
}
function Absolute($x){
  return abs($x);
}
function Logarithm($x){
  return log10($x);
}
function NaturalLogarithm($x){
  return log($x);
}
function Sinx($x){
  return sin($x);
}
function Cosx($x){
  return cos($x);
}
function Tanx($x){
  return tan($x);
}
function Asinx($x){
  return asin($x);
}
function Acosx($x){
  return acos($x);
}
function Atanx($x){
  return atan($x);
}
function Atan2x($y, $x){
  $a = 0.0;

  if($x > 0.0){
    $a = Atanx($y/$x);
  }else if($x < 0.0 && $y >= 0.0){
    $a = Atanx($y/$x) + M_PI;
  }else if($x < 0.0 && $y < 0.0){
    $a = Atanx($y/$x) - M_PI;
  }else if($x == 0.0 && $y > 0.0){
    $a = M_PI/2.0;
  }else if($x == 0.0 && $y < 0.0){
    $a = -M_PI/2.0;
  }

  return $a;
}
function Squareroot($x){
  return sqrt($x);
}
function Expx($x){
  return exp($x);
}
function DivisibleBy($a, $b){
  return (($a%$b) == 0.0);
}
function Combinations($n, $k){
  return Factorial($n)/(Factorial($n - $k)*Factorial($k));
}
function EpsilonCompareApproximateDigits($a, $b, $digits){
  if($a < 0.0 && $b < 0.0 || $a > 0.0 && $b > 0.0){
    if($a < 0.0 && $b < 0.0){
      $a = -$a;
      $b = -$b;
    }
    $ad = log10($a);
    $bd = log10($b);
    $d = max($ad, $bd);
    $epsilon = 10.0**($d - $digits);
    $ret = abs($a - $b) > $epsilon;
  }else{
    $ret = false;
  }

  return $ret;
}
function EpsilonCompare($a, $b, $epsilon){
  return abs($a - $b) < $epsilon;
}
function GreatestCommonDivisor($a, $b){
  for(; $b != 0.0; ){
    $t = $b;
    $b = $a%$b;
    $a = $t;
  }

  return $a;
}
function IsInteger($a){
  return ($a - floor($a)) == 0.0;
}
function GreatestCommonDivisorWithCheck($a, $b, $gcdReference){
  if(IsInteger($a) && IsInteger($b)){
    $gcd = GreatestCommonDivisor($a, $b);
    $gcdReference->numberValue = $gcd;
    $success = true;
  }else{
    $success = false;
  }

  return $success;
}
function LeastCommonMultiple($a, $b){
  if($a > 0.0 && $b > 0.0){
    $lcm = abs($a*$b)/GreatestCommonDivisor($a, $b);
  }else{
    $lcm = 0.0;
  }

  return $lcm;
}
function Sign($a){
  if($a > 0.0){
    $s = 1.0;
  }else if($a < 0.0){
    $s = -1.0;
  }else{
    $s = 0.0;
  }

  return $s;
}
function Maxx($a, $b){
  return max($a, $b);
}
function Minx($a, $b){
  return min($a, $b);
}
function Power($a, $b){
  return $a**$b;
}
function CreateBooleanReference($value){
  $ref = new stdClass();
  $ref->booleanValue = $value;

  return $ref;
}
function CreateBooleanArrayReference(&$value){
  $ref = new stdClass();
  $ref->booleanArray = $value;

  return $ref;
}
function CreateBooleanArrayReferenceLengthValue($length, $value){
  $ref = new stdClass();
  $ref->booleanArray = array_fill(0, $length, 0);

  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $ref->booleanArray[$i] = $value;
  }

  return $ref;
}
function FreeBooleanArrayReference($booleanArrayReference){
  unset($booleanArrayReference->booleanArray);
  unset($booleanArrayReference);
}
function CreateCharacterReference($value){
  $ref = new stdClass();
  $ref->characterValue = $value;

  return $ref;
}
function CreateNumberReference($value){
  $ref = new stdClass();
  $ref->numberValue = $value;

  return $ref;
}
function CreateNumberArrayReference(&$value){
  $ref = new stdClass();
  $ref->numberArray = $value;

  return $ref;
}
function CreateNumberArrayReferenceLengthValue($length, $value){
  $ref = new stdClass();
  $ref->numberArray = array_fill(0, $length, 0);

  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $ref->numberArray[$i] = $value;
  }

  return $ref;
}
function FreeNumberArrayReference($numberArrayReference){
  unset($numberArrayReference->numberArray);
  unset($numberArrayReference);
}
function CreateStringReference(&$value){
  $ref = new stdClass();
  $ref->string = $value;

  return $ref;
}
function CreateStringReferenceLengthValue($length, $value){
  $ref = new stdClass();
  $ref->string = array_fill(0, $length, 0);

  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $ref->string[$i] = $value;
  }

  return $ref;
}
function FreeStringReference($stringReference){
  unset($stringReference->string);
  unset($stringReference);
}
function CreateStringArrayReference(&$strings){
  $ref = new stdClass();
  $ref->stringArray = $strings;

  return $ref;
}
function CreateStringArrayReferenceLengthValue($length, &$value){
  $ref = new stdClass();
  $ref->stringArray = array_fill(0, $length, 0);

  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $ref->stringArray[$i] = CreateStringReference($value);
  }

  return $ref;
}
function FreeStringArrayReference($stringArrayReference){
  for($i = 0.0; $i < count($stringArrayReference->stringArray); $i = $i + 1.0){
    unset($stringArrayReference->stringArray[$i]);
  }
  unset($stringArrayReference->stringArray);
  unset($stringArrayReference);
}
function CreatePriorityQueueBTNumbers(){

  $q = new stdClass();
  $q->heap = CreateDynamicArrayNumbers();

  return $q;
}
function FreePriorityQueueBTNumbers($q){
  FreeDynamicArrayNumbers($q->heap);
  unset($q);
}
function PeekPriorityQueueBTNumbers($q, $keyReference){

  if( !IsEmptyPriorityQueueBTNumbers($q) ){
    $keyReference->numberValue = DynamicArrayNumbersIndex($q->heap, 0.0);
    $found = true;
  }else{
    $found = false;
  }

  return $found;
}
function InsertIntoPriorityQueueBTNumbers($q, $key){
  DynamicArrayAddNumber($q->heap, $key);

  if(SizeOfPriorityQueueBTNumbers($q) >= 2.0){
    SiftUpPriorityQueueBTNumbers($q, $q->heap->length - 1.0);
  }
}
function PopPriorityQueueBTNumbers($q, $keyReference){

  $found = PeekPriorityQueueBTNumbers($q, $keyReference);

  if($found){
    DeleteTopPriorityQueueBTNumbers($q);
  }

  return $found;
}
function DeleteTopPriorityQueueBTNumbers($q){

  $found = IsEmptyPriorityQueueBTNumbers($q);

  if( !IsEmptyPriorityQueueBTNumbers($q) ){
    $last = $q->heap->length - 1.0;
    SwapElementsOfArray($q->heap->array, 0.0, $last);

    DynamicArrayRemoveNumber($q->heap, $last);

    SiftDownPriorityQueueBTNumbers($q, 0.0);
  }

  return $found;
}
function ArrayToPriorityQueueBTNumbers(&$keys){

  $q = CreatePriorityQueueBTNumbers();

  for($i = 0.0; $i < count($keys); $i = $i + 1.0){
    InsertIntoPriorityQueueBTNumbers($q, $keys[$i]);
  }

  return $q;
}
function SizeOfPriorityQueueBTNumbers($q){
  return $q->heap->length;
}
function IsEmptyPriorityQueueBTNumbers($q){
  return $q->heap->length == 0.0;
}
function SiftUpPriorityQueueBTNumbers($q, $index){

  $done = false;
  for(;  !$done  && $index != 0.0; ){
    $parent = floor(($index - 1.0)/2.0);

    $iKey = DynamicArrayNumbersIndex($q->heap, $index);
    $pKey = DynamicArrayNumbersIndex($q->heap, $parent);

    if($iKey > $pKey){
      SwapElementsOfArray($q->heap->array, $index, $parent);
    }else{
      $done = true;
    }

    $index = $parent;
  }
}
function SiftDownPriorityQueueBTNumbers($q, $index){

  $size = SizeOfPriorityQueueBTNumbers($q);

  $done = false;
  for(;  !$done ; ){
    $parent = $index;
    $c1 = 2.0*$parent + 1.0;
    $c2 = 2.0*$parent + 2.0;

    $pKey = DynamicArrayNumbersIndex($q->heap, $parent);
    $c1Key = DynamicArrayNumbersIndex($q->heap, $c1);
    $c2Key = DynamicArrayNumbersIndex($q->heap, $c2);

    if($c1Key > $pKey && $c1 < $size || $c2Key > $pKey && $c2 < $size){
      if($c1Key >= $c2Key && $c1 < $size){
        SwapElementsOfArray($q->heap->array, $c1, $parent);
        $index = $c1;
      }else if($c1Key <= $c2Key && $c2 < $size){
        SwapElementsOfArray($q->heap->array, $c2, $parent);
        $index = $c2;
      }else{
        $done = true;
      }
    }else{
      $done = true;
    }
  }
}
function CreatePriorityQueueBTNumKeyValue(){

  $q = new stdClass();
  $q->heapKey = CreateDynamicArrayNumbers();
  $q->heapValue = CreateDynamicArrayNumbers();

  return $q;
}
function FreePriorityQueueBTNumKeyValue($q){
  FreeDynamicArrayNumbers($q->heapKey);
  FreeDynamicArrayNumbers($q->heapValue);
  unset($q);
}
function PeekPriorityQueueBTNumKeyValue($q, $keyReference, $valueReference){

  if( !IsEmptyPriorityQueueBTNumKeyValue($q) ){
    $keyReference->numberValue = DynamicArrayNumbersIndex($q->heapKey, 0.0);
    $valueReference->numberValue = DynamicArrayNumbersIndex($q->heapValue, 0.0);
    $found = true;
  }else{
    $found = false;
  }

  return $found;
}
function InsertIntoPriorityQueueBTNumKeyValue($q, $key, $value){
  DynamicArrayAddNumber($q->heapKey, $key);
  DynamicArrayAddNumber($q->heapValue, $value);

  if(SizeOfPriorityQueueBTNumKeyValue($q) >= 2.0){
    SiftUpPriorityQueueBTNumKeyValue($q, $q->heapKey->length - 1.0);
  }
}
function PopPriorityQueueBTNumKeyValue($q, $keyReference, $valueReference){

  $found = PeekPriorityQueueBTNumKeyValue($q, $keyReference, $valueReference);

  if($found){
    DeleteTopPriorityQueueBTNumKeyValue($q);
  }

  return $found;
}
function DeleteTopPriorityQueueBTNumKeyValue($q){

  $found = IsEmptyPriorityQueueBTNumKeyValue($q);

  if( !IsEmptyPriorityQueueBTNumKeyValue($q) ){
    $last = $q->heapKey->length - 1.0;
    SwapElementsOfArray($q->heapKey->array, 0.0, $last);
    SwapElementsOfArray($q->heapValue->array, 0.0, $last);

    DynamicArrayRemoveNumber($q->heapKey, $last);
    DynamicArrayRemoveNumber($q->heapValue, $last);

    SiftDownPriorityQueueBTNumKeyValue($q, 0.0);
  }

  return $found;
}
function ArrayToPriorityQueueBTNumKeyValue(&$keys, &$values){

  $q = CreatePriorityQueueBTNumKeyValue();

  for($i = 0.0; $i < count($keys); $i = $i + 1.0){
    InsertIntoPriorityQueueBTNumKeyValue($q, $keys[$i], $values[$i]);
  }

  return $q;
}
function SizeOfPriorityQueueBTNumKeyValue($q){
  return $q->heapKey->length;
}
function IsEmptyPriorityQueueBTNumKeyValue($q){
  return $q->heapKey->length == 0.0;
}
function SiftUpPriorityQueueBTNumKeyValue($q, $index){

  $done = false;
  for(;  !$done  && $index != 0.0; ){
    $parent = floor(($index - 1.0)/2.0);

    $iKey = DynamicArrayNumbersIndex($q->heapKey, $index);
    $pKey = DynamicArrayNumbersIndex($q->heapKey, $parent);

    if($iKey > $pKey){
      SwapElementsOfArray($q->heapKey->array, $index, $parent);
      SwapElementsOfArray($q->heapValue->array, $index, $parent);
    }else{
      $done = true;
    }

    $index = $parent;
  }
}
function SiftDownPriorityQueueBTNumKeyValue($q, $index){

  $size = SizeOfPriorityQueueBTNumKeyValue($q);

  $c1Key = 0.0;
  $c2Key = 0.0;
  $done = false;
  for(;  !$done ; ){
    $parent = $index;
    $c1 = 2.0*$parent + 1.0;
    $c2 = 2.0*$parent + 2.0;

    $pKey = DynamicArrayNumbersIndex($q->heapKey, $parent);
    if($c1 < $size){
      $c1Key = DynamicArrayNumbersIndex($q->heapKey, $c1);
    }
    if($c2 < $size){
      $c2Key = DynamicArrayNumbersIndex($q->heapKey, $c2);
    }

    if($c1Key > $pKey && $c1 < $size || $c2Key > $pKey && $c2 < $size){
      if($c1Key >= $c2Key && $c1 < $size){
        SwapElementsOfArray($q->heapKey->array, $c1, $parent);
        SwapElementsOfArray($q->heapValue->array, $c1, $parent);
        $index = $c1;
      }else if($c1Key <= $c2Key && $c2 < $size){
        SwapElementsOfArray($q->heapKey->array, $c2, $parent);
        SwapElementsOfArray($q->heapValue->array, $c2, $parent);
        $index = $c2;
      }else{
        $done = true;
      }
    }else{
      $done = true;
    }
  }
}
function CreateLinkedListNumbers(){

  $ll = new stdClass();
  $ll->first = new stdClass();
  $ll->last = $ll->first;
  $ll->last->end = true;

  return $ll;
}
function &CreateLinkedListNumbersArray($length){

  $lls = array_fill(0, $length, 0);
  for($i = 0.0; $i < count($lls); $i = $i + 1.0){
    $lls[$i] = CreateLinkedListNumbers();
  }

  return $lls;
}
function LinkedListAddNumber($ll, $value){
  $ll->last->end = false;
  $ll->last->value = $value;
  $ll->last->next = new stdClass();
  $ll->last->next->end = true;
  $ll->last = $ll->last->next;
}
function LinkedListNumbersLength($ll){

  $l = 0.0;
  $node = $ll->first;
  for(;  !$node->end ; ){
    $node = $node->next;
    $l = $l + 1.0;
  }

  return $l;
}
function LinkedListNumbersIndex($ll, $index){

  $node = $ll->first;
  for($i = 0.0; $i < $index; $i = $i + 1.0){
    $node = $node->next;
  }

  return $node->value;
}
function LinkedListInsertNumber($ll, $index, $value){

  if($index == 0.0){
    $tmp = $ll->first;
    $ll->first = new stdClass();
    $ll->first->next = $tmp;
    $ll->first->value = $value;
    $ll->first->end = false;
  }else{
    $node = $ll->first;
    for($i = 0.0; $i < $index - 1.0; $i = $i + 1.0){
      $node = $node->next;
    }

    $tmp = $node->next;
    $node->next = new stdClass();
    $node->next->next = $tmp;
    $node->next->value = $value;
    $node->next->end = false;
  }
}
function LinkedListSet($ll, $index, $value){

  $node = $ll->first;
  for($i = 0.0; $i < $index; $i = $i + 1.0){
    $node = $node->next;
  }

  $node->next->value = $value;
}
function LinkedListRemoveNumber($ll, $index){

  $node = $ll->first;
  $prev = $ll->first;

  for($i = 0.0; $i < $index; $i = $i + 1.0){
    $prev = $node;
    $node = $node->next;
  }

  if($index == 0.0){
    $ll->first = $prev->next;
  }

  $prev->next = $prev->next->next;
}
function FreeLinkedListNumbers($ll){

  $node = $ll->first;

  for(;  !$node->end ; ){
    $prev = $node;
    $node = $node->next;
    unset($prev);
  }

  unset($node);
}
function FreeLinkedListNumbersArray(&$lls){

  for($i = 0.0; $i < count($lls); $i = $i + 1.0){
    FreeLinkedListNumbers($lls[$i]);
  }
  unset($lls);
}
function &LinkedListNumbersToArray($ll){

  $node = $ll->first;

  $length = LinkedListNumbersLength($ll);

  $array = array_fill(0, $length, 0);

  for($i = 0.0; $i < $length; $i = $i + 1.0){
    $array[$i] = $node->value;
    $node = $node->next;
  }

  return $array;
}
function ArrayToLinkedListNumbers(&$array){

  $ll = CreateLinkedListNumbers();

  for($i = 0.0; $i < count($array); $i = $i + 1.0){
    LinkedListAddNumber($ll, $array[$i]);
  }

  return $ll;
}
function LinkedListNumbersEqual($a, $b){

  $an = $a->first;
  $bn = $b->first;

  $equal = true;
  $done = false;
  for(; $equal &&  !$done ; ){
    if($an->end == $bn->end){
      if($an->end){
        $done = true;
      }else if($an->value == $bn->value){
        $an = $an->next;
        $bn = $bn->next;
      }else{
        $equal = false;
      }
    }else{
      $equal = false;
    }
  }

  return $equal;
}
function CreateDynamicArrayNumbers(){

  $da = new stdClass();
  $da->array = array_fill(0, 10.0, 0);
  $da->length = 0.0;

  return $da;
}
function CreateDynamicArrayNumbersWithInitialCapacity($capacity){

  $da = new stdClass();
  $da->array = array_fill(0, $capacity, 0);
  $da->length = 0.0;

  return $da;
}
function DynamicArrayAddNumber($da, $value){
  if($da->length == count($da->array)){
    DynamicArrayNumbersIncreaseSize($da);
  }

  $da->array[$da->length] = $value;
  $da->length = $da->length + 1.0;
}
function DynamicArrayNumbersIncreaseSize($da){

  $newLength = round(count($da->array)*3.0/2.0);
  $newArray = array_fill(0, $newLength, 0);

  for($i = 0.0; $i < count($da->array); $i = $i + 1.0){
    $newArray[$i] = $da->array[$i];
  }

  unset($da->array);

  $da->array = $newArray;
}
function DynamicArrayNumbersDecreaseSizeNecessary($da){

  $needsDecrease = false;

  if($da->length > 10.0){
    $needsDecrease = $da->length <= round(count($da->array)*2.0/3.0);
  }

  return $needsDecrease;
}
function DynamicArrayNumbersDecreaseSize($da){

  $newLength = round(count($da->array)*2.0/3.0);
  $newArray = array_fill(0, $newLength, 0);

  for($i = 0.0; $i < count($da->array); $i = $i + 1.0){
    $newArray[$i] = $da->array[$i];
  }

  unset($da->array);

  $da->array = $newArray;
}
function DynamicArrayNumbersIndex($da, $index){
  return $da->array[$index];
}
function DynamicArrayNumbersLength($da){
  return $da->length;
}
function DynamicArrayInsertNumber($da, $index, $value){

  if($da->length == count($da->array)){
    DynamicArrayNumbersIncreaseSize($da);
  }

  for($i = $da->length; $i >= $index; $i = $i - 1.0){
    $da->array[$i + 1.0] = $da->array[$i];
  }

  $da->array[$index] = $value;

  $da->length = $da->length + 1.0;
}
function DynamicArraySet($da, $index, $value){
  $da->array[$index] = $value;
}
function DynamicArrayRemoveNumber($da, $index){

  for($i = $index; $i < $da->length - 1.0; $i = $i + 1.0){
    $da->array[$i] = $da->array[$i + 1.0];
  }

  $da->length = $da->length - 1.0;

  if(DynamicArrayNumbersDecreaseSizeNecessary($da)){
    DynamicArrayNumbersDecreaseSize($da);
  }
}
function FreeDynamicArrayNumbers($da){
  unset($da->array);
  unset($da);
}
function &DynamicArrayNumbersToArray($da){

  $array = array_fill(0, $da->length, 0);

  for($i = 0.0; $i < $da->length; $i = $i + 1.0){
    $array[$i] = $da->array[$i];
  }

  return $array;
}
function ArrayToDynamicArrayNumbersWithOptimalSize(&$array){

  /*
         c = 10*(3/2)^n
         log(c) = log(10*(3/2)^n)
         log(c) = log(10) + log((3/2)^n)
         log(c) = 1 + log((3/2)^n)
         log(c) - 1 = log((3/2)^n)
         log(c) - 1 = n*log(3/2)
         n = (log(c) - 1)/log(3/2)
         */
  $c = count($array);
  $n = (log($c) - 1.0)/log(3.0/2.0);
  $newCapacity = floor($n) + 1.0;

  $da = CreateDynamicArrayNumbersWithInitialCapacity($newCapacity);

  for($i = 0.0; $i < count($array); $i = $i + 1.0){
    $da->array[$i] = $array[$i];
  }

  return $da;
}
function ArrayToDynamicArrayNumbers(&$array){

  $da = new stdClass();
  $da->array = CopyNumberArray($array);
  $da->length = count($array);

  return $da;
}
function DynamicArrayNumbersEqual($a, $b){

  $equal = true;
  if($a->length == $b->length){
    for($i = 0.0; $i < $a->length && $equal; $i = $i + 1.0){
      if($a->array[$i] != $b->array[$i]){
        $equal = false;
      }
    }
  }else{
    $equal = false;
  }

  return $equal;
}
function DynamicArrayNumbersToLinkedList($da){

  $ll = CreateLinkedListNumbers();

  for($i = 0.0; $i < $da->length; $i = $i + 1.0){
    LinkedListAddNumber($ll, $da->array[$i]);
  }

  return $ll;
}
function LinkedListToDynamicArrayNumbers($ll){

  $node = $ll->first;

  $da = new stdClass();
  $da->length = LinkedListNumbersLength($ll);

  $da->array = array_fill(0, $da->length, 0);

  for($i = 0.0; $i < $da->length; $i = $i + 1.0){
    $da->array[$i] = $node->value;
    $node = $node->next;
  }

  return $da;
}
function &AddNumber(&$list, $a){

  $newlist = array_fill(0, count($list) + 1.0, 0);
  for($i = 0.0; $i < count($list); $i = $i + 1.0){
    $newlist[$i] = $list[$i];
  }
  $newlist[count($list)] = $a;
		
  unset($list);
		
  return $newlist;
}
function AddNumberRef($list, $i){
  $list->numberArray = AddNumber($list->numberArray, $i);
}
function &RemoveNumber(&$list, $n){

  $newlist = array_fill(0, count($list) - 1.0, 0);

  if($n >= 0.0 && $n < count($list)){
    for($i = 0.0; $i < count($list); $i = $i + 1.0){
      if($i < $n){
        $newlist[$i] = $list[$i];
      }
      if($i > $n){
        $newlist[$i - 1.0] = $list[$i];
      }
    }

    unset($list);
  }else{
    unset($newlist);
  }
		
  return $newlist;
}
function GetNumberRef($list, $i){
  return $list->numberArray[$i];
}
function RemoveNumberRef($list, $i){
  $list->numberArray = RemoveNumber($list->numberArray, $i);
}
function &AddString(&$list, $a){

  $newlist = array_fill(0, count($list) + 1.0, 0);

  for($i = 0.0; $i < count($list); $i = $i + 1.0){
    $newlist[$i] = $list[$i];
  }
  $newlist[count($list)] = $a;
		
  unset($list);
		
  return $newlist;
}
function AddStringRef($list, $i){
  $list->stringArray = AddString($list->stringArray, $i);
}
function &RemoveString(&$list, $n){

  $newlist = array_fill(0, count($list) - 1.0, 0);

  if($n >= 0.0 && $n < count($list)){
    for($i = 0.0; $i < count($list); $i = $i + 1.0){
      if($i < $n){
        $newlist[$i] = $list[$i];
      }
      if($i > $n){
        $newlist[$i - 1.0] = $list[$i];
      }
    }

    unset($list);
  }else{
    unset($newlist);
  }
		
  return $newlist;
}
function GetStringRef($list, $i){
  return $list->stringArray[$i];
}
function RemoveStringRef($list, $i){
  $list->stringArray = RemoveString($list->stringArray, $i);
}
function &AddBoolean(&$list, $a){

  $newlist = array_fill(0, count($list) + 1.0, 0);
  for($i = 0.0; $i < count($list); $i = $i + 1.0){
    $newlist[$i] = $list[$i];
  }
  $newlist[count($list)] = $a;
		
  unset($list);
		
  return $newlist;
}
function AddBooleanRef($list, $i){
  $list->booleanArray = AddBoolean($list->booleanArray, $i);
}
function &RemoveBoolean(&$list, $n){

  $newlist = array_fill(0, count($list) - 1.0, 0);

  if($n >= 0.0 && $n < count($list)){
    for($i = 0.0; $i < count($list); $i = $i + 1.0){
      if($i < $n){
        $newlist[$i] = $list[$i];
      }
      if($i > $n){
        $newlist[$i - 1.0] = $list[$i];
      }
    }

    unset($list);
  }else{
    unset($newlist);
  }
		
  return $newlist;
}
function GetBooleanRef($list, $i){
  return $list->booleanArray[$i];
}
function RemoveDecimalRef($list, $i){
  $list->booleanArray = RemoveBoolean($list->booleanArray, $i);
}
function &AddCharacter(&$list, $a){

  $newlist = array_fill(0, count($list) + 1.0, 0);
  for($i = 0.0; $i < count($list); $i = $i + 1.0){
    $newlist[$i] = $list[$i];
  }
  $newlist[count($list)] = $a;
		
  unset($list);
		
  return $newlist;
}
function AddCharacterRef($list, $i){
  $list->string = AddCharacter($list->string, $i);
}
function &RemoveCharacter(&$list, $n){

  $newlist = array_fill(0, count($list) - 1.0, 0);

  if($n >= 0.0 && $n < count($list)){
    for($i = 0.0; $i < count($list); $i = $i + 1.0){
      if($i < $n){
        $newlist[$i] = $list[$i];
      }
      if($i > $n){
        $newlist[$i - 1.0] = $list[$i];
      }
    }

    unset($list);
  }else{
    unset($newlist);
  }

  return $newlist;
}
function GetCharacterRef($list, $i){
  return $list->string[$i];
}
function RemoveCharacterRef($list, $i){
  $list->string = RemoveCharacter($list->string, $i);
}
function TreeHeight($tree){

  $heightSet = false;
  $height = 0.0;

  for($i = 0.0; $i < count($tree->branches); $i = $i + 1.0){
    $branchHeight = TreeHeight($tree->branches[$i]);
    if( !$heightSet ){
      $height = $branchHeight;
      $heightSet = true;
    }else if($branchHeight > $height){
      $height = $branchHeight;
    }
  }

  if(count($tree->branches) == 0.0){
    $height = 0.0;
  }else{
    $height = $height + 1.0;
  }

  return $height;
}
function TreeNumberOfNodes($tree){

  $nodes = 0.0;

  for($i = 0.0; $i < count($tree->branches); $i = $i + 1.0){
    $nodes = $nodes + TreeNumberOfNodes($tree->branches[$i]);
  }

  return $nodes + 1.0;
}
function AssertFalse($b, $failures){
  if($b){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertTrue($b, $failures){
  if( !$b ){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertEquals($a, $b, $failures){
  if($a != $b){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertBooleansEqual($a, $b, $failures){
  if($a != $b){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertCharactersEqual($a, $b, $failures){
  if($a != $b){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertStringEquals(&$a, &$b, $failures){
  if( !StringsEqual($a, $b) ){
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertNumberArraysEqual(&$a, &$b, $failures){

  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a); $i = $i + 1.0){
      AssertEquals($a[$i], $b[$i], $failures);
    }
  }else{
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertBooleanArraysEqual(&$a, &$b, $failures){

  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a); $i = $i + 1.0){
      AssertBooleansEqual($a[$i], $b[$i], $failures);
    }
  }else{
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}
function AssertStringArraysEqual(&$a, &$b, $failures){

  if(count($a) == count($b)){
    for($i = 0.0; $i < count($a); $i = $i + 1.0){
      AssertStringEquals($a[$i]->string, $b[$i]->string, $failures);
    }
  }else{
    $failures->numberValue = $failures->numberValue + 1.0;
  }
}

