package edu.unm.dragonfly;

import java.util.*;

public class GeneticTSP {

    private static final int ELETISM_COUNT = 1;
    private static final int TOURNAMENT_SIZE = 5;
    private static final int CROSSOVERS = 2;
    private static final double MUTATION_RATE = 0.015;

    private static final Random RAND = new Random(System.currentTimeMillis());

    public static class Tour {

        private final List<PointImpl> points;
        private double distance;

        public Tour(List<PointImpl> points) {
            this.points = points;
            this.distance = distance(points);
        }

        public static Tour generate(List<PointImpl> points) {
            List<PointImpl> shuffledPoints = new ArrayList<>(points);
            Collections.shuffle(shuffledPoints);
            return new Tour(shuffledPoints);
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

        public List<PointImpl> getPoints() {
            return points;
        }
    }

    public static class Population{

        private final List<Tour> individuals;
        private final Tour mostFit;

        private Population(List<Tour> individuals) {
            this.individuals = individuals;
            this.mostFit = individuals.stream().max(Comparator.comparingDouble(Tour::getFitness)).get();
        }


        public static Population generate(List<PointImpl> points, int size) {

            List<Tour> individuals = new ArrayList<>();

            for(int i = 0; i < size; i++) {
                individuals.add(Tour.generate(points));
            }

            return new Population(individuals);
        }

        public Tour getMostFit() {
            return mostFit;
        }

        public List<Tour> getIndividuals() {
            return individuals;
        }
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

    private static double distance(List<PointImpl> points) {
        double distance = 0;
        for(int i = 0; i < points.size() - 1; i++) {
            distance += distance(points.get(i), points.get(i + 1));
        }
        return distance;
    }

    private static double distance(PointImpl start, PointImpl end) {
        double deltax = start.getX() - end.getX();
        double deltay = start.getY() - end.getY();
        return Math.sqrt((deltax * deltax) + (deltay * deltay));
    }

    public static Population evolve(Population population) {
        List<Tour> nextEvolution = new ArrayList<>();

        // Elitism
        List<Tour> sortedTours = new ArrayList<>(population.getIndividuals());
        Collections.sort(sortedTours, new Comparator<Tour>() {
            @Override
            public int compare(Tour o1, Tour o2) {
                return Double.compare(o1.getDistance(), o2.getDistance());
            }
        });
        for(int i = 0; i < ELETISM_COUNT; i++) {
            nextEvolution.add(sortedTours.get(i));
        }

        // Tournament Selection, Crossover, and Mutation
        for(int i = ELETISM_COUNT-1; i < population.getIndividuals().size(); i++) {
            Tour parent1 = tournament(population.getIndividuals());
            Tour parent2 = tournament(population.getIndividuals());
            Tour child = new Tour(mutate(crossover(parent1, parent2)));
            nextEvolution.add(child);
        }

        return new Population(nextEvolution);
    }

    private static List<PointImpl> mutate(List<PointImpl> individual) {
        List<PointImpl> mutatedIndividual = new ArrayList<>(individual);
        for(int i = 0; i < individual.size(); i++) {
            if(RAND.nextDouble() < MUTATION_RATE) {
                int j = RAND.nextInt(individual.size());
                Collections.swap(mutatedIndividual, i, j);
            }
        }
        return mutatedIndividual;
    }

    private static List<PointImpl> crossover(Tour parent1, Tour parent2) {
        PointImpl[] points = new PointImpl[parent1.size()];

        boolean parentOne = false;
        List<Integer> crossovers = new ArrayList<>(parent1.size());
        for(int i = 0; i < CROSSOVERS; i++) {
            crossovers.add(RAND.nextInt(parent1.size()));
        }

        for(int i = 0; i < parent1.size(); i++) {
            if(crossovers.contains(i)) {
                parentOne = !parentOne;
            }
            if(parentOne) {
                points[i] = parent1.getPoints().get(i);
            }
        }

        for(int i = 0; i < parent2.size(); i++) {
            PointImpl parentTwoPoint = parent2.getPoints().get(i);
            if(!Arrays.asList(points).contains(parentTwoPoint)) {
                for(int j = 0; j < points.length; j++) {
                    if(points[j] == null) {
                        points[j] = parentTwoPoint;
                        break;
                    }
                }
            }
        }

        return Arrays.asList(points);
    }

    private static Tour tournament(List<Tour> individuals) {
        Tour victor = individuals.get(RAND.nextInt(individuals.size()));
        for(int i = 0; i < TOURNAMENT_SIZE - 1; i++) {
            Tour challenger = individuals.get(RAND.nextInt(individuals.size()));
            if(challenger.getFitness() > victor.getFitness()) {
                victor = challenger;
            }
        }
        return victor;
    }

    public static void main(String[] args) {

        List<PointImpl> points = new ArrayList<>();
        for(int i = 0; i < 200; i++) {
            points.add(new PointImpl(RAND.nextDouble() * 100, RAND.nextDouble() * 100));
        }

        double length = GeneticTSP.distance(points);
        System.out.println("Distance: " + length);

        Population population = Population.generate(points, 500);

        for(int i = 0; i < 5000; i++) {
            long start = System.currentTimeMillis();
            population = GeneticTSP.evolve(population);

            System.out.println("Evolution " + i + " " +
                    "took: " + (System.currentTimeMillis() - start) + "ms, " +
                    "distance: " + population.getMostFit().getDistance());
        }
    }
}