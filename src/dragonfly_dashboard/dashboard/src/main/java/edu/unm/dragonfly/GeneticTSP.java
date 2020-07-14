package edu.unm.dragonfly;

import java.util.*;

public class GeneticTSP {

    private static final int ELETISM_COUNT = 1;
    private static final int TOURNAMENT_SIZE = 5;
    private static final double MUTATION_RATE = 0.015;

    private static final Random RAND = new Random(System.currentTimeMillis());

    public interface DistanceMetric<P> {

        double distance(List<P> points);
    }

    public static class Tour<P> {

        private final List<P> points;
        private final double distance;

        public Tour(List<P> points, DistanceMetric<P> distance) {
            this.points = points;
            this.distance = distance.distance(points);
        }

        public static <P> Tour<P> generate(List<P> points, DistanceMetric<P> metric) {
            List<P> shuffledPoints = new ArrayList<P>(points);
            Collections.shuffle(shuffledPoints);
            return new Tour<P>(shuffledPoints, metric);
        }

        public double getDistance() {
            return distance;
        }

        public double getFitness() {
            return 1 / distance;
        }

        public int size() {
            return points.size();
        }

        public List<P> getPoints() {
            return points;
        }
    }

    private static class ArrayPool<P> {
        private final int arraySize;
        private final List<List<P>> available = new ArrayList<>();

        public ArrayPool(int arraySize) {
            this.arraySize = arraySize;
        }

        public List<P> checkout() {
            if(!available.isEmpty()) {
                return available.remove(0);
            } else {
                List<P> creation = new ArrayList<>(arraySize);
                for(int i = 0; i < arraySize; i++) {
                    creation.add(null);
                }

                return creation;
            }
        }

        public void checkin(List<P> obsoleted) {
            available.add(obsoleted);
        }
    }

    public static class Population<P>{

        private final List<Tour<P>> individuals;
        private final ArrayPool<P> pool;
        private final DistanceMetric<P> distanceMetric;
        private final Tour<P> mostFit;

        private Population(List<Tour<P>> individuals, ArrayPool<P> pool, DistanceMetric<P> distanceMetric) {
            this.individuals = individuals;
            this.pool = pool;
            this.distanceMetric = distanceMetric;
            this.mostFit = individuals.stream().max(Comparator.comparingDouble(Tour::getFitness)).get();
        }


        public static <P> Population<P> generate(List<P> points, DistanceMetric<P> metric, int size) {

            ArrayPool<P> pool = new ArrayPool<>(points.size());

            List<Tour<P>> individuals = new ArrayList<>();

            for(int i = 0; i < size; i++) {
                individuals.add(Tour.generate(points, metric));
            }

            return new Population<P>(individuals, pool, metric);
        }

        public Tour<P> getMostFit() {
            return mostFit;
        }

        public List<Tour<P>> getIndividuals() {
            return individuals;
        }

        public DistanceMetric<P> getMetric() {
            return distanceMetric;
        }

        public ArrayPool<P> getPool() {
            return pool;
        }
    }

    public static <P> Population<P> evolve(Population<P> population) {
        List<Tour<P>> nextEvolution = new ArrayList<>();

        // Elitism
        Collections.sort(population.getIndividuals(), Comparator.comparingDouble(Tour::getDistance));
        for(int i = 0; i < ELETISM_COUNT; i++) {
            nextEvolution.add(population.getIndividuals().get(i));
        }

        // Tournament Selection, Crossover, and Mutation
        for(int i = ELETISM_COUNT-1; i < population.getIndividuals().size(); i++) {
            Tour<P> parent1 = tournament(population.getIndividuals());
            Tour<P> parent2 = tournament(population.getIndividuals());
            Tour<P> child = new Tour<P>(mutate(crossover(parent1, parent2, population.getPool())), population.getMetric());
            nextEvolution.add(child);
        }
        
        for(int i = ELETISM_COUNT; i < population.getIndividuals().size(); i++) {
            population.getPool().checkin(population.getIndividuals().get(i).getPoints());
        }

        return new Population<P>(nextEvolution, population.getPool(), population.getMetric());
    }

    private static <P> List<P> mutate(List<P> individual) {
        for(int i = 0; i < individual.size(); i++) {
            if(RAND.nextDouble() < MUTATION_RATE) {
                int j = RAND.nextInt(individual.size());
                Collections.swap(individual, i, j);
            }
        }
        return individual;
    }

    private static <P> List<P> crossover(Tour<P> parent1, Tour<P> parent2, ArrayPool<P> pool) {
        List<P> points = pool.checkout();

        boolean parentOne = false;
        int crossoverOne = RAND.nextInt(parent1.size());
        int crossoverTwo = RAND.nextInt(parent1.size());

        for(int i = 0; i < parent1.size(); i++) {
            if(i == crossoverOne || i == crossoverTwo) {
                parentOne = !parentOne;
            }
            if(parentOne) {
                points.set(i, parent1.getPoints().get(i));
            } else {
                points.set(i, null);
            }
        }

        for(int i = 0; i < parent2.size(); i++) {
            P parentTwoPoint = parent2.getPoints().get(i);
            if(!points.contains(parentTwoPoint)) {
                for(int j = 0; j < points.size(); j++) {
                    if(points.get(j) == null) {
                        points.set(j, parentTwoPoint);
                        break;
                    }
                }
            }
        }

        return points;
    }

    private static <P> Tour<P> tournament(List<Tour<P>> individuals) {
        Tour<P> victor = individuals.get(RAND.nextInt(individuals.size()));
        for(int i = 0; i < TOURNAMENT_SIZE - 1; i++) {
            Tour<P> challenger = individuals.get(RAND.nextInt(individuals.size()));
            if(challenger.getFitness() > victor.getFitness()) {
                victor = challenger;
            }
        }
        return victor;
    }

    public static class PointImpl {
        private final double x;
        private final double y;

        public PointImpl(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }

    public static void main(String[] args) {

        List<PointImpl> points = new ArrayList<>();
        for(int i = 0; i < 200; i++) {
            points.add(new PointImpl(RAND.nextDouble() * 100, RAND.nextDouble() * 100));
        }

        Population<PointImpl> population = Population.generate(points, new DistanceMetric<PointImpl>() {
            @Override
            public double distance(List<PointImpl> points) {
                double distance = 0;
                for(int i = 0; i < points.size() - 1; i++) {
                    double deltax = points.get(i).getX() - points.get(i + 1).getX();
                    double deltay = points.get(i).getY() - points.get(i + 1).getY();
                    distance += Math.sqrt((deltax * deltax) + (deltay * deltay));
                }
                return distance;
            }
        }, 500);

        for(int i = 0; i < 5000; i++) {
            long start = System.currentTimeMillis();
            population = GeneticTSP.evolve(population);

            System.out.println("Evolution " + i + " " +
                    "took: " + (System.currentTimeMillis() - start) + "ms, " +
                    "distance: " + population.getMostFit().getDistance());
        }
    }
}